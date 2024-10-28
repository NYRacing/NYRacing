/*
 * Code generated for Simulink model VehCtrlMdel241025_2018b_amkspdlimit.
 *
 * FILE    : VehCtrlMdel241025_2018b_amkspdlimit.h
 *
 * VERSION : 1.305
 *
 * DATE    : Fri Oct 25 15:32:30 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#ifndef RTW_HEADER_VehCtrlMdel241025_2018b_amkspdlimit_h_
#define RTW_HEADER_VehCtrlMdel241025_2018b_amkspdlimit_h_
#include <math.h>
#include "MPC5744P.h"
#include "Std_Types.h"
#include "can.h"
#include "flash.h"
#include "crc.h"
#ifndef VehCtrlMdel241025_2018b_amkspdlimit_COMMON_INCLUDES_
# define VehCtrlMdel241025_2018b_amkspdlimit_COMMON_INCLUDES_
#include <string.h>
#include <math.h>
#include "rtwtypes.h"
#include "can_message.h"
#endif                /* VehCtrlMdel241025_2018b_amkspdlimit_COMMON_INCLUDES_ */

#include "VehCtrlMdel241025_2018b_amkspdlimit_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmStepTask
# define rtmStepTask(rtm, idx)         ((rtm)->Timing.TaskCounters.TID[(idx)] == 0)
#endif

#ifndef rtmTaskCounter
# define rtmTaskCounter(rtm, idx)      ((rtm)->Timing.TaskCounters.TID[(idx)])
#endif

#define VehCtrlMdel241025_2018b_amkspdlimit_M (VehCtrlMdel241025_2018b_amks_M)
#define EnableInterrupts()             asm(" wrteei 1")
#define DisableInterrupts()            asm(" wrteei 0")

/* user code (top of export header file) */
#include "can_message.h"

/* Block states (default storage) for system '<S8>/Timer1' */
typedef struct {
  real_T x;                            /* '<S8>/Timer1' */
  struct {
    uint_T is_c5_VehCtrlMdel241025_2018b_a:2;/* '<S8>/Timer1' */
    uint_T is_active_c5_VehCtrlMdel241025_:1;/* '<S8>/Timer1' */
  } bitsForTID3;
} DW_Timer1_VehCtrlMdel241025_2_T;

/* Block states (default storage) for system '<S130>/Timer' */
typedef struct {
  real_T x;                            /* '<S130>/Timer' */
  struct {
    uint_T is_c21_VehCtrlMdel241025_2018b_:2;/* '<S130>/Timer' */
    uint_T is_active_c21_VehCtrlMdel241025:1;/* '<S130>/Timer' */
  } bitsForTID3;
} DW_Timer_VehCtrlMdel241025_20_T;

/* Block signals (default storage) */
typedef struct {
  CAN_DATATYPE CANPack1;               /* '<S361>/CAN Pack1' */
  CAN_DATATYPE CANPack1_b;             /* '<S360>/CAN Pack1' */
  CAN_DATATYPE CANPack1_d;             /* '<S359>/CAN Pack1' */
  CAN_DATATYPE CANUnPackMessage4;      /* '<S190>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_g;    /* '<S184>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_j;    /* '<S161>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_n;    /* '<S172>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_p;    /* '<S170>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_e;    /* '<S142>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_c;    /* '<S154>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_b;    /* '<S152>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_l;    /* '<S203>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_ja;   /* '<S132>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_h;    /* '<S198>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_i;    /* '<S181>/CANUnPackMessage4' */
  real_T Switch1;                      /* '<S360>/Switch1' */
  real_T Switch;                       /* '<S360>/Switch' */
  real_T Switch1_l;                    /* '<S359>/Switch1' */
  real_T MCFL_TorqueLimitP;            /* '<S359>/Switch' */
  real_T Exit;                         /* '<S358>/Timer3' */
  real_T Exit_h;                       /* '<S358>/Timer2' */
  real_T LEDOn;                        /* '<S366>/Chart' */
  real_T Exit_i;                       /* '<S288>/Timer2' */
  real_T Exit_l;                       /* '<S288>/Timer1' */
  real_T Exit_a;                       /* '<S287>/Timer3' */
  real_T Exit_lh;                      /* '<S287>/Timer2' */
  real_T Exit_lh4;                     /* '<S287>/Timer1' */
  real_T Exit_c;                       /* '<S287>/Timer' */
  real_T Exit_hj;                      /* '<S222>/Timer3' */
  real_T Exit_o;                       /* '<S222>/Timer2' */
  real_T Exit_is;                      /* '<S222>/Timer1' */
  real_T Exit_le;                      /* '<S222>/Timer' */
  real_T Exit_n;                       /* '<S221>/Timer' */
  real_T Exit_on;                      /* '<S219>/Timer' */
  real_T ModeSW_o;
  real_T AMKSWITCH_bx;
  real_T ignition_d;
  real_T Exit_iy;                      /* '<S130>/Timer2' */
  real_T low_VOL;                      /* '<S190>/CAN Unpack' */
  real_T MCU_Temp_error;               /* '<S190>/CAN Unpack' */
  real_T Mode;                         /* '<S190>/CAN Unpack' */
  real_T motorTemp_error;              /* '<S190>/CAN Unpack' */
  real_T overCurrent;                  /* '<S190>/CAN Unpack' */
  real_T overpower;                    /* '<S190>/CAN Unpack' */
  real_T overvol;                      /* '<S190>/CAN Unpack' */
  real_T Precharge;                    /* '<S190>/CAN Unpack' */
  real_T Reslove_error;                /* '<S190>/CAN Unpack' */
  real_T MCFR_bDerating;               /* '<S161>/CAN Unpack' */
  real_T MCFR_bQuitDCOn;               /* '<S161>/CAN Unpack' */
  real_T MCFR_bReserve;                /* '<S161>/CAN Unpack' */
  real_T MCFR_bWarn;                   /* '<S161>/CAN Unpack' */
  real_T MCFR_DiagnosticNum;           /* '<S170>/CAN Unpack' */
  real_T MCFL_bDerating;               /* '<S142>/CAN Unpack' */
  real_T MCFL_bReserve;                /* '<S142>/CAN Unpack' */
  real_T MCFL_bWarn;                   /* '<S142>/CAN Unpack' */
  real_T MCFL_DiagnosticNum;           /* '<S152>/CAN Unpack' */
  real_T CANUnpack_o1;                 /* '<S198>/CAN Unpack' */
  real_T CANUnpack_o3;                 /* '<S198>/CAN Unpack' */
  real_T CANUnpack_o5;                 /* '<S198>/CAN Unpack' */
  real_T CANUnpack_o6;                 /* '<S198>/CAN Unpack' */
  real_T Switch11;                     /* '<S113>/Switch11' */
  real_T errorReset;                   /* '<S113>/Chart2' */
  real_T Exit_d;                       /* '<S8>/Timer2' */
  real_T Exit_g;                       /* '<S8>/Timer1' */
  real_T DYC_Enable_OUT;               /* '<S7>/Chart' */
  real_T TCSR_Enable_OUT;              /* '<S7>/Chart' */
  real_T TCSF_Enable_OUT;              /* '<S7>/Chart' */
  uint32_T CANReceive_o3;              /* '<S397>/CANReceive' */
  uint32_T CANReceive_o3_l;            /* '<S382>/CANReceive' */
  uint32_T CANReceive1_o3;             /* '<S127>/CANReceive1' */
  uint32_T CANReceive3_o3;             /* '<S127>/CANReceive3' */
  uint32_T CANReceive3_o3_e;           /* '<S137>/CANReceive3' */
  uint32_T CANReceive1_o3_n;           /* '<S137>/CANReceive1' */
  uint32_T CANReceive2_o3;             /* '<S137>/CANReceive2' */
  uint32_T CANReceive3_o3_i;           /* '<S138>/CANReceive3' */
  uint32_T CANReceive1_o3_h;           /* '<S138>/CANReceive1' */
  uint32_T CANReceive2_o3_j;           /* '<S138>/CANReceive2' */
  uint32_T CANReceive3_o3_c;           /* '<S129>/CANReceive3' */
  uint32_T CANReceive3_o3_m;           /* '<S123>/CANReceive3' */
  uint32_T CANReceive3_o3_cz;          /* '<S128>/CANReceive3' */
  uint32_T CANReceive3_o3_l;           /* '<S126>/CANReceive3' */
  real32_T CastToBoolean4;             /* '<S361>/Cast To Boolean4' */
  real32_T CastToBoolean6;             /* '<S361>/Cast To Boolean6' */
  real32_T VCU_SpdCmd_Emrax;           /* '<S7>/Constant13' */
  int32_T DataTypeConversion2;         /* '<S361>/Data Type Conversion2' */
  uint16_T CastToSingle1;              /* '<S362>/Cast To Single1' */
  uint16_T HV_volt;                    /* '<S130>/Acc4' */
  uint8_T CANReceive_o2;               /* '<S397>/CANReceive' */
  uint8_T CANReceive_o4[8];            /* '<S397>/CANReceive' */
  uint8_T CANReceive_o5;               /* '<S397>/CANReceive' */
  uint8_T CANReceive_o2_p;             /* '<S382>/CANReceive' */
  uint8_T CANReceive_o4_i[8];          /* '<S382>/CANReceive' */
  uint8_T CANReceive_o5_l;             /* '<S382>/CANReceive' */
  uint8_T CANTransmit;                 /* '<S389>/CANTransmit' */
  uint8_T CANPackMessage[8];           /* '<S361>/CANPackMessage' */
  uint8_T CANTransmit_k;               /* '<S361>/CANTransmit' */
  uint8_T CANPackMessage_f[8];         /* '<S360>/CANPackMessage' */
  uint8_T CANTransmit_l;               /* '<S360>/CANTransmit' */
  uint8_T CANPackMessage_h[8];         /* '<S359>/CANPackMessage' */
  uint8_T CANTransmit_c;               /* '<S359>/CANTransmit' */
  uint8_T CANReceive1_o2;              /* '<S127>/CANReceive1' */
  uint8_T CANReceive1_o4[8];           /* '<S127>/CANReceive1' */
  uint8_T CANReceive1_o5;              /* '<S127>/CANReceive1' */
  uint8_T CANReceive3_o2;              /* '<S127>/CANReceive3' */
  uint8_T CANReceive3_o4[8];           /* '<S127>/CANReceive3' */
  uint8_T CANReceive3_o5;              /* '<S127>/CANReceive3' */
  uint8_T CANReceive3_o2_l;            /* '<S137>/CANReceive3' */
  uint8_T CANReceive3_o4_l[8];         /* '<S137>/CANReceive3' */
  uint8_T CANReceive3_o5_a;            /* '<S137>/CANReceive3' */
  uint8_T CANReceive1_o2_l;            /* '<S137>/CANReceive1' */
  uint8_T CANReceive1_o4_c[8];         /* '<S137>/CANReceive1' */
  uint8_T CANReceive1_o5_a;            /* '<S137>/CANReceive1' */
  uint8_T CANReceive2_o2;              /* '<S137>/CANReceive2' */
  uint8_T CANReceive2_o4[6];           /* '<S137>/CANReceive2' */
  uint8_T CANReceive2_o5;              /* '<S137>/CANReceive2' */
  uint8_T CANReceive3_o2_a;            /* '<S138>/CANReceive3' */
  uint8_T CANReceive3_o4_g[8];         /* '<S138>/CANReceive3' */
  uint8_T CANReceive3_o5_an;           /* '<S138>/CANReceive3' */
  uint8_T CANReceive1_o2_o;            /* '<S138>/CANReceive1' */
  uint8_T CANReceive1_o4_j[8];         /* '<S138>/CANReceive1' */
  uint8_T CANReceive1_o5_j;            /* '<S138>/CANReceive1' */
  uint8_T CANReceive2_o2_p;            /* '<S138>/CANReceive2' */
  uint8_T CANReceive2_o4_k[6];         /* '<S138>/CANReceive2' */
  uint8_T CANReceive2_o5_e;            /* '<S138>/CANReceive2' */
  uint8_T CANReceive3_o2_p;            /* '<S129>/CANReceive3' */
  uint8_T CANReceive3_o4_k[8];         /* '<S129>/CANReceive3' */
  uint8_T CANReceive3_o5_d;            /* '<S129>/CANReceive3' */
  uint8_T CANReceive3_o2_m;            /* '<S123>/CANReceive3' */
  uint8_T CANReceive3_o4_lg[8];        /* '<S123>/CANReceive3' */
  uint8_T CANReceive3_o5_b;            /* '<S123>/CANReceive3' */
  uint8_T CANReceive3_o2_ma;           /* '<S128>/CANReceive3' */
  uint8_T CANReceive3_o4_i[8];         /* '<S128>/CANReceive3' */
  uint8_T CANReceive3_o5_m;            /* '<S128>/CANReceive3' */
  uint8_T CANReceive3_o2_k;            /* '<S126>/CANReceive3' */
  uint8_T CANReceive3_o4_p[8];         /* '<S126>/CANReceive3' */
  uint8_T CANReceive3_o5_de;           /* '<S126>/CANReceive3' */
  boolean_T AND;                       /* '<S358>/AND' */
  boolean_T led;                       /* '<S370>/Chart1' */
  boolean_T Drive_ready;               /* '<S130>/SwitchInput' */
  boolean_T SwitchInput1;              /* '<S130>/SwitchInput1' */
  boolean_T out2_c;                    /* '<S130>/SwitchInput3' */
  boolean_T SwitchInput4;              /* '<S130>/SwitchInput4' */
  boolean_T TSAL_SW_IN_i2;
  boolean_T out2_h;
  boolean_T HV_voltValid_kx;
  boolean_T MCFL_DCOn_setpoints_o;     /* '<S113>/Switch4' */
  boolean_T VehReady;                  /* '<S113>/Chart2' */
  boolean_T MCFL_DCEnable;             /* '<S113>/Chart2' */
  boolean_T MCFR_TorqueOn;             /* '<S113>/Chart2' */
  boolean_T MCFL_TorqueOn;             /* '<S113>/Chart2' */
  boolean_T AMKMCFL_InverterOn;        /* '<S113>/Chart2' */
  boolean_T bWaterPumpON;
  boolean_T aWaterPumpON;
  boolean_T RelationalOperator1;       /* '<S73>/Relational Operator1' */
  boolean_T RelationalOperator1_c;     /* '<S72>/Relational Operator1' */
} B_VehCtrlMdel241025_2018b_amk_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DelayInput2_DSTATE;           /* '<S273>/Delay Input2' */
  real_T DelayInput2_DSTATE_b;         /* '<S274>/Delay Input2' */
  real_T DelayInput2_DSTATE_h;         /* '<S275>/Delay Input2' */
  real_T DelayInput2_DSTATE_n;         /* '<S276>/Delay Input2' */
  real_T DelayInput2_DSTATE_l;         /* '<S243>/Delay Input2' */
  real_T UnitDelay_DSTATE;             /* '<S242>/Unit Delay' */
  real_T DelayInput2_DSTATE_m;         /* '<S244>/Delay Input2' */
  real_T UnitDelay1_DSTATE;            /* '<S242>/Unit Delay1' */
  real_T DelayInput2_DSTATE_k;         /* '<S245>/Delay Input2' */
  real_T UnitDelay2_DSTATE;            /* '<S242>/Unit Delay2' */
  real_T UnitDelay_DSTATE_l;           /* '<S69>/Unit Delay' */
  real_T UnitDelay_DSTATE_n;           /* '<S7>/Unit Delay' */
  real_T DelayInput2_DSTATE_p;         /* '<S42>/Delay Input2' */
  real_T UnitDelay_DSTATE_h;           /* '<S10>/Unit Delay' */
  real_T UnitDelay1_DSTATE_c;          /* '<S10>/Unit Delay1' */
  real_T DelayInput2_DSTATE_lt;        /* '<S47>/Delay Input2' */
  real_T DelayInput2_DSTATE_hj;        /* '<S41>/Delay Input2' */
  real_T DelayInput2_DSTATE_pd;        /* '<S43>/Delay Input2' */
  real_T DelayInput2_DSTATE_nk;        /* '<S46>/Delay Input2' */
  real_T x;                            /* '<S130>/Timer2' */
  real_T x_j;                          /* '<S130>/Timer1' */
  real_T b;                            /* '<S7>/Chart' */
  real_T DYC_flag;                     /* '<S7>/Chart' */
  real32_T UnitDelay_DSTATE_p;         /* '<S303>/Unit Delay' */
  real32_T DelayInput2_DSTATE_n2;      /* '<S307>/Delay Input2' */
  real32_T UnitDelay_DSTATE_j;         /* '<S296>/Unit Delay' */
  real32_T UnitDelay_DSTATE_pj;        /* '<S304>/Unit Delay' */
  real32_T DelayInput2_DSTATE_e;       /* '<S310>/Delay Input2' */
  real32_T UnitDelay1_DSTATE_n;        /* '<S296>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_a;         /* '<S305>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hk;      /* '<S313>/Delay Input2' */
  real32_T UnitDelay2_DSTATE_l;        /* '<S296>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_nc;        /* '<S306>/Unit Delay' */
  real32_T DelayInput2_DSTATE_c;       /* '<S316>/Delay Input2' */
  real32_T UnitDelay3_DSTATE;          /* '<S296>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_lh;        /* '<S323>/Unit Delay' */
  real32_T DelayInput2_DSTATE_i;       /* '<S327>/Delay Input2' */
  real32_T UnitDelay4_DSTATE;          /* '<S297>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_a0;        /* '<S297>/Unit Delay' */
  real32_T UnitDelay_DSTATE_ap;        /* '<S324>/Unit Delay' */
  real32_T DelayInput2_DSTATE_el;      /* '<S330>/Delay Input2' */
  real32_T UnitDelay5_DSTATE;          /* '<S297>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_a;        /* '<S297>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_o;         /* '<S325>/Unit Delay' */
  real32_T DelayInput2_DSTATE_pdc;     /* '<S333>/Delay Input2' */
  real32_T UnitDelay6_DSTATE;          /* '<S297>/Unit Delay6' */
  real32_T UnitDelay2_DSTATE_c;        /* '<S297>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_ah;        /* '<S326>/Unit Delay' */
  real32_T DelayInput2_DSTATE_mt;      /* '<S336>/Delay Input2' */
  real32_T UnitDelay7_DSTATE;          /* '<S297>/Unit Delay7' */
  real32_T UnitDelay3_DSTATE_d;        /* '<S297>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_lha;       /* '<S216>/Unit Delay' */
  real32_T UnitDelay4_DSTATE_m;        /* '<S285>/Unit Delay4' */
  real32_T UnitDelay1_DSTATE_k;        /* '<S216>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_o;        /* '<S285>/Unit Delay1' */
  real32_T UnitDelay3_DSTATE_p;        /* '<S288>/Unit Delay3' */
  real32_T UnitDelay1_DSTATE_d;        /* '<S288>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_f;        /* '<S288>/Unit Delay2' */
  real32_T UnitDelay4_DSTATE_mn;       /* '<S288>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_d;         /* '<S285>/Unit Delay' */
  real32_T UnitDelay2_DSTATE_i;        /* '<S285>/Unit Delay2' */
  real32_T DelayInput2_DSTATE_g;       /* '<S348>/Delay Input2' */
  real32_T DelayInput2_DSTATE_a;       /* '<S346>/Delay Input2' */
  real32_T DelayInput2_DSTATE_f;       /* '<S352>/Delay Input2' */
  real32_T UnitDelay_DSTATE_ncs;       /* '<S345>/Unit Delay' */
  real32_T DelayInput2_DSTATE_hu;      /* '<S347>/Delay Input2' */
  real32_T DelayInput2_DSTATE_l2;      /* '<S253>/Delay Input2' */
  real32_T UnitDelay_DSTATE_o2;        /* '<S219>/Unit Delay' */
  real32_T DelayInput2_DSTATE_j;       /* '<S254>/Delay Input2' */
  real32_T UnitDelay1_DSTATE_aq;       /* '<S219>/Unit Delay1' */
  real32_T DelayInput2_DSTATE_l4;      /* '<S44>/Delay Input2' */
  real32_T DelayInput2_DSTATE_j3;      /* '<S45>/Delay Input2' */
  real32_T UnitDelay4_DSTATE_j;        /* '<S10>/Unit Delay4' */
  real32_T UnitDelay2_DSTATE_j;        /* '<S10>/Unit Delay2' */
  real32_T UnitDelay5_DSTATE_k;        /* '<S10>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_f;        /* '<S31>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_n5;       /* '<S87>/Unit Delay1' */
  real32_T UnitDelay4_DSTATE_i;        /* '<S31>/Unit Delay4' */
  real32_T UnitDelay5_DSTATE_i;        /* '<S31>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_f;         /* '<S31>/Unit Delay' */
  real32_T UnitDelay2_DSTATE_jr;       /* '<S31>/Unit Delay2' */
  real32_T UnitDelay1_DSTATE_i;        /* '<S98>/Unit Delay1' */
  real32_T UnitDelay4_DSTATE_b;        /* '<S32>/Unit Delay4' */
  real32_T UnitDelay1_DSTATE_h;        /* '<S78>/Unit Delay1' */
  real32_T UnitDelay4_DSTATE_l;        /* '<S30>/Unit Delay4' */
  real32_T UnitDelay5_DSTATE_ip;       /* '<S30>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_nr;        /* '<S30>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_g;        /* '<S30>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_b;        /* '<S30>/Unit Delay2' */
  real32_T UnitDelay5_DSTATE_l;        /* '<S32>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_b;         /* '<S32>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_gu;       /* '<S32>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_g;        /* '<S32>/Unit Delay2' */
  real32_T DelayInput2_DSTATE_cd;      /* '<S19>/Delay Input2' */
  real32_T DelayInput2_DSTATE_hn;      /* '<S20>/Delay Input2' */
  real32_T DelayInput2_DSTATE_ib;      /* '<S21>/Delay Input2' */
  int32_T sfEvent;                     /* '<S113>/Chart2' */
  uint32_T previousTicks;              /* '<S370>/Chart1' */
  uint32_T previousTicks_e;            /* '<S366>/Chart' */
  uint32_T previousTicks_m;            /* '<S358>/Chart' */
  uint32_T Subsystem_PREV_T;           /* '<S4>/Subsystem' */
  uint32_T FunctionCallSubsystem_PREV_T;/* '<S4>/Function-Call Subsystem' */
  uint32_T previousTicks_j;            /* '<S221>/Chart' */
  uint32_T previousTicks_g;            /* '<S113>/Chart2' */
  uint32_T MoTrqReq_PREV_T;            /* '<S1>/MoTrqReq' */
  int_T CANPack1_ModeSignalID;         /* '<S361>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_g;       /* '<S360>/CAN Pack1' */
  int_T CANPack1_ModeSignalID_f;       /* '<S359>/CAN Pack1' */
  int_T CANUnpack_ModeSignalID;        /* '<S190>/CAN Unpack' */
  int_T CANUnpack_StatusPortID;        /* '<S190>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_b;      /* '<S184>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_m;      /* '<S184>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_a;      /* '<S161>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_o;      /* '<S161>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_n;      /* '<S172>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_f;      /* '<S172>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_p;      /* '<S170>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_e;      /* '<S170>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_c;      /* '<S142>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_ok;     /* '<S142>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_m;      /* '<S154>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_j;      /* '<S154>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_i;      /* '<S152>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_mj;     /* '<S152>/CAN Unpack' */
  int_T CANUnpack1_ModeSignalID;       /* '<S203>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID;       /* '<S203>/CAN Unpack1' */
  int_T CANUnpack1_ModeSignalID_m;     /* '<S132>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID_n;     /* '<S132>/CAN Unpack1' */
  int_T CANUnpack_ModeSignalID_f;      /* '<S198>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_k;      /* '<S198>/CAN Unpack' */
  struct {
    uint_T is_VehStat:4;               /* '<S113>/Chart2' */
    uint_T is_AMKDCready:4;            /* '<S113>/Chart2' */
    uint_T is_c27_VehCtrlMdel241025_2018b_:3;/* '<S366>/Chart' */
    uint_T is_c32_VehCtrlMdel241025_2018b_:2;/* '<S370>/Chart1' */
    uint_T is_StateA:2;                /* '<S366>/Chart' */
    uint_T is_StateB:2;                /* '<S366>/Chart' */
    uint_T is_StateC:2;                /* '<S366>/Chart' */
    uint_T is_c24_VehCtrlMdel241025_2018b_:2;/* '<S358>/Chart' */
    uint_T is_STATEON:2;               /* '<S358>/Chart' */
    uint_T is_STATEOFF:2;              /* '<S358>/Chart' */
    uint_T is_c28_VehCtrlMdel241025_2018b_:2;/* '<S221>/Chart' */
    uint_T is_STATEON_d:2;             /* '<S221>/Chart' */
    uint_T is_c30_VehCtrlMdel241025_2018b_:2;/* '<S130>/Timer2' */
    uint_T is_c23_VehCtrlMdel241025_2018b_:2;/* '<S130>/Timer1' */
    uint_T is_BeeperStat:2;            /* '<S113>/Chart2' */
    uint_T is_AMKDCon:2;               /* '<S113>/Chart2' */
    uint_T is_MCDCEnable:2;            /* '<S113>/Chart2' */
    uint_T is_MC_TorqueCUT:2;          /* '<S113>/Chart2' */
    uint_T is_AMKCANenable:2;          /* '<S113>/Chart2' */
    uint_T is_MCFL_InverterOn:2;       /* '<S113>/Chart2' */
    uint_T is_MCFR_InverterOn:2;       /* '<S113>/Chart2' */
    uint_T is_B:2;                     /* '<S7>/Chart' */
    uint_T is_C:2;                     /* '<S7>/Chart' */
    uint_T is_D:2;                     /* '<S7>/Chart' */
    uint_T is_E:2;                     /* '<S7>/Chart' */
    uint_T is_active_c32_VehCtrlMdel241025:1;/* '<S370>/Chart1' */
    uint_T is_active_c27_VehCtrlMdel241025:1;/* '<S366>/Chart' */
    uint_T is_active_c24_VehCtrlMdel241025:1;/* '<S358>/Chart' */
    uint_T is_active_c28_VehCtrlMdel241025:1;/* '<S221>/Chart' */
    uint_T is_active_c30_VehCtrlMdel241025:1;/* '<S130>/Timer2' */
    uint_T is_active_c23_VehCtrlMdel241025:1;/* '<S130>/Timer1' */
    uint_T is_active_c1_VehCtrlMdel241025_:1;/* '<S113>/Chart2' */
    uint_T is_active_VehStat:1;        /* '<S113>/Chart2' */
    uint_T is_active_BeeperStat:1;     /* '<S113>/Chart2' */
    uint_T is_active_AMKDCon:1;        /* '<S113>/Chart2' */
    uint_T is_active_MCDCEnable:1;     /* '<S113>/Chart2' */
    uint_T is_active_MC_TorqueCUT:1;   /* '<S113>/Chart2' */
    uint_T is_active_AMKDCready:1;     /* '<S113>/Chart2' */
    uint_T is_active_Output:1;         /* '<S113>/Chart2' */
    uint_T is_active_AMKCANenable:1;   /* '<S113>/Chart2' */
    uint_T is_active_MCFL_InverterOn:1;/* '<S113>/Chart2' */
    uint_T is_active_MCFR_InverterOn:1;/* '<S113>/Chart2' */
    uint_T is_active_c7_VehCtrlMdel241025_:1;/* '<S7>/Chart' */
  } bitsForTID3;

  uint16_T UnitDelay1_DSTATE_fm;       /* '<S217>/Unit Delay1' */
  uint16_T UnitDelay_DSTATE_k;         /* '<S217>/Unit Delay' */
  uint16_T temporalCounter_i2;         /* '<S113>/Chart2' */
  boolean_T DelayInput1_DSTATE;        /* '<S365>/Delay Input1' */
  boolean_T UnitDelay3_DSTATE_f;       /* '<S285>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE_a;      /* '<S262>/Delay Input1' */
  boolean_T UnitDelay_DSTATE_pl;       /* '<S71>/Unit Delay' */
  boolean_T UnitDelay3_DSTATE_i;       /* '<S10>/Unit Delay3' */
  boolean_T UnitDelay6_DSTATE_b;       /* '<S10>/Unit Delay6' */
  boolean_T UnitDelay1_DSTATE_gl;      /* '<S86>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_e;       /* '<S31>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE_j;      /* '<S85>/Delay Input1' */
  boolean_T UnitDelay1_DSTATE_e;       /* '<S97>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_a;       /* '<S32>/Unit Delay3' */
  boolean_T UnitDelay1_DSTATE_dp;      /* '<S77>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE_ip;      /* '<S30>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE_b;      /* '<S76>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_e;      /* '<S96>/Delay Input1' */
  uint8_T temporalCounter_i1;          /* '<S370>/Chart1' */
  uint8_T temporalCounter_i1_j;        /* '<S366>/Chart' */
  uint8_T temporalCounter_i1_p;        /* '<S358>/Chart' */
  uint8_T temporalCounter_i1_h;        /* '<S221>/Chart' */
  uint8_T temporalCounter_i1_f;        /* '<S113>/Chart2' */
  boolean_T Subsystem_RESET_ELAPS_T;   /* '<S4>/Subsystem' */
  boolean_T FunctionCallSubsystem_RESET_ELA;/* '<S4>/Function-Call Subsystem' */
  boolean_T MoTrqReq_RESET_ELAPS_T;    /* '<S1>/MoTrqReq' */
  boolean_T Subsystem_MODE;            /* '<S358>/Subsystem' */
  boolean_T EnabledSubsystem1_MODE;    /* '<S358>/Enabled Subsystem1' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer3_f;/* '<S358>/Timer3' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer2_h;/* '<S358>/Timer2' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer2_j;/* '<S288>/Timer2' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer1_p;/* '<S288>/Timer1' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer3_i;/* '<S287>/Timer3' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer2_g;/* '<S287>/Timer2' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer1_m;/* '<S287>/Timer1' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer_o;/* '<S287>/Timer' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer3;/* '<S222>/Timer3' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer2_l;/* '<S222>/Timer2' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer1_n;/* '<S222>/Timer1' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer_b;/* '<S222>/Timer' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer_p;/* '<S221>/Timer' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer_k;/* '<S219>/Timer' */
  DW_Timer_VehCtrlMdel241025_20_T sf_Timer_a;/* '<S217>/Timer' */
  DW_Timer_VehCtrlMdel241025_20_T sf_Timer;/* '<S130>/Timer' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer2;/* '<S8>/Timer2' */
  DW_Timer1_VehCtrlMdel241025_2_T sf_Timer1;/* '<S8>/Timer1' */
} DW_VehCtrlMdel241025_2018b_am_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real_T MultiportSwitch[2];     /* '<S71>/Multiport Switch' */
} ConstB_VehCtrlMdel241025_2018_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: [229.8500	229.8500	229.8500	229.8500	229.8500	229.8500	229.8500	229.8500	229.8500	229.8500	229.8500	229.8500	220.3846	204.6429	191	179.0625	168.5294	159.1667	150.7895	143.2500	136.4286	130.2273	124.5652	119.3750	114.6000	110.1923	106.1111]
   * Referenced by: '<S7>/228RWD1'
   */
  real_T u28RWD1_tableData[27];

  /* Pooled Parameter (Expression: [200;400;600;800;1000;1200;1400;1600;1800;2000;2200;2400;2600;2800;3000;3200;3400;3600;3800;4000;4200;4400;4600;4800;5000;5200;5400])
   * Referenced by:
   *   '<S7>/228RWD'
   *   '<S7>/228RWD1'
   *   '<S10>/228'
   */
  real_T pooled4[27];

  /* Expression: [229.8500	229.8500	229.8500	229.8500	229.8500	229.8500	229.8500	229.8500	229.8500	229.8500	229.8500	229.8500	229.4600	229.1600	228.8600	223.8281	210.6618	198.9583	188.4868	179.0625	170.5357	162.7841	155.7065	149.2188	143.2500	137.7404	132.6389]
   * Referenced by: '<S7>/228RWD'
   */
  real_T u28RWD_tableData[27];

  /* Pooled Parameter (Expression: [21;21;21;21;21;21;21;21;21;21;21;21;21;21;20;17.5;15;12.5;8;0])
   * Referenced by:
   *   '<S10>/AMK'
   *   '<S10>/AMK1'
   */
  real_T pooled7[20];

  /* Pooled Parameter (Expression: [0;1000;2000;3000;4000;5000;6000;7000;8000;9000;10000;11000;12000;13000;14000;15000;16000;17000;18000;19000])
   * Referenced by:
   *   '<S7>/AMK2'
   *   '<S7>/AMK3'
   *   '<S10>/AMK'
   *   '<S10>/AMK1'
   */
  real_T pooled8[20];

  /* Expression: [229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.46;229.16;228.86;228.86;228.86;228.86;228.86;227.86;227.86;226.87;225.86;224.86;220.86;218.87;215.87]
   * Referenced by: '<S10>/228'
   */
  real_T u28_tableData[27];

  /* Pooled Parameter (Expression: [21	21	21	21	21	21	19.8958	17.0536	14.9219	13.2639	11.9375	10.8523	9.9479	9.1827	8.5268	7.9583	7.4609	7.0221	6.6319	0])
   * Referenced by:
   *   '<S7>/AMK2'
   *   '<S7>/AMK3'
   */
  real_T pooled17[20];

  /* Pooled Parameter (Expression: [0,0,0.2,0.4,0.6,0.8,1,1])
   * Referenced by:
   *   '<S8>/2-D Lookup Table1'
   *   '<S8>/2-D Lookup Table3'
   *   '<S8>/2-D Lookup Table4'
   */
  real_T pooled18[8];

  /* Pooled Parameter (Expression: [35,40,42,44,46,48,50,55])
   * Referenced by:
   *   '<S8>/2-D Lookup Table1'
   *   '<S8>/2-D Lookup Table4'
   */
  real_T pooled19[8];

  /* Expression: [45,50,55,60,65,70,75,80]
   * Referenced by: '<S8>/2-D Lookup Table3'
   */
  real_T uDLookupTable3_bp01Data[8];

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
  real32_T pooled33[231];

  /* Pooled Parameter (Expression: single([0,10,20,30,40,50,60,70,80,90,100]);)
   * Referenced by:
   *   '<S29>/4WD_Table'
   *   '<S29>/RWD_Table'
   */
  real32_T pooled34[11];

  /* Pooled Parameter (Expression: single([5.400000095367432,10.800000190734863,16.200000762939453,21.600000381469727,27,32.400001525878906,37.79999923706055,43.20000076293945,48.599998474121094,54,59.400001525878906,64.80000305175781,70.19999694824219,75.5999984741211,81,86.4000015258789,91.80000305175781,97.19999694824219,102.5999984741211,108,114]);)
   * Referenced by:
   *   '<S29>/4WD_Table'
   *   '<S29>/RWD_Table'
   */
  real32_T pooled35[21];

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
  real32_T pooled62[4];

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
  real32_T pooled63[4];

  /* Pooled Parameter (Expression: [0.4,0.4,1.2,1.2])
   * Referenced by:
   *   '<S30>/VehicleStableTarget_mps'
   *   '<S30>/VehicleStableTarget_mps1'
   *   '<S31>/VehicleStableTarget_mps'
   *   '<S31>/VehicleStableTarget_mps1'
   *   '<S32>/VehicleStableTarget_mps'
   *   '<S32>/VehicleStableTarget_mps1'
   */
  real32_T pooled69[4];

  /* Expression: single([20,0]);
   * Referenced by: '<S7>/BrakeCompensateCoefFront1'
   */
  real32_T BrakeCompensateCoefFront1_table[2];

  /* Expression: single([550,1700]);
   * Referenced by: '<S7>/BrakeCompensateCoefFront1'
   */
  real32_T BrakeCompensateCoefFront1_bp01D[2];

  /* Pooled Parameter (Expression: single([500,4500]);)
   * Referenced by: '<S217>/1-D Lookup Table1'
   */
  real32_T pooled74[2];

  /* Pooled Parameter (Expression: single([0,100]);)
   * Referenced by:
   *   '<S217>/1-D Lookup Table3'
   *   '<S217>/1-D Lookup Table4'
   */
  real32_T pooled75[2];

  /* Expression: single([2589,2754])
   * Referenced by: '<S217>/1-D Lookup Table4'
   */
  real32_T uDLookupTable4_bp01Data[2];

  /* Expression: single([2471,2642])
   * Referenced by: '<S217>/1-D Lookup Table3'
   */
  real32_T uDLookupTable3_bp01Data_b[2];

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
   * Referenced by: '<S219>/1-D Lookup Table'
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
   * Referenced by: '<S219>/1-D Lookup Table'
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
   * Referenced by: '<S219>/1-D Lookup Table1'
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
   * Referenced by: '<S219>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_bp01Data_h[24];

  /* Pooled Parameter (Expression: )
   * Referenced by:
   *   '<S29>/4WD_Table'
   *   '<S29>/RWD_Table'
   */
  uint32_T pooled86[2];

  /* Computed Parameter: uDLookupTable1_maxIndex
   * Referenced by: '<S10>/2-D Lookup Table1'
   */
  uint32_T uDLookupTable1_maxIndex[2];
} ConstP_VehCtrlMdel241025_2018_T;

/* Real-time Model Data Structure */
struct tag_RTM_VehCtrlMdel241025_201_T {
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
extern B_VehCtrlMdel241025_2018b_amk_T VehCtrlMdel241025_2018b_amksp_B;

/* Block states (default storage) */
extern DW_VehCtrlMdel241025_2018b_am_T VehCtrlMdel241025_2018b_amks_DW;
extern const ConstB_VehCtrlMdel241025_2018_T VehCtrlMdel241025_2018b__ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_VehCtrlMdel241025_2018_T VehCtrlMdel241025_2018b__ConstP;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real_T Gear_Trs;                /* '<S361>/Switch2' */
extern real_T Mode_Trs;                /* '<S361>/Switch3' */
extern real_T KeyPressed;
                       /* '<S215>/BusConversion_InsertedFor_Out1_at_inport_0' */
extern real_T AMKFL_Current;           /* '<S220>/Switch' */
extern real_T AMKFR_Current;           /* '<S220>/Switch1' */
extern real_T EmraxPwr;                /* '<S220>/Product2' */
extern real_T Trq_CUT;                 /* '<S217>/Timer' */
extern real_T AMKSWITCH;               /* '<S130>/Timer1' */
extern real_T ignition;                /* '<S130>/Timer' */
extern real_T L12V_error;              /* '<S190>/CAN Unpack' */
extern real_T alarm;                   /* '<S190>/CAN Unpack' */
extern real_T controller_ready;        /* '<S190>/CAN Unpack' */
extern real_T selfcheck;               /* '<S190>/CAN Unpack' */
extern real_T RPM;                     /* '<S190>/CAN Unpack' */
extern real_T trq;                     /* '<S190>/CAN Unpack' */
extern real_T AC_current;              /* '<S184>/CAN Unpack' */
extern real_T DC_current;              /* '<S184>/CAN Unpack' */
extern real_T MCU_Temp;                /* '<S184>/CAN Unpack' */
extern real_T motor_Temp;              /* '<S184>/CAN Unpack' */
extern real_T voltage;                 /* '<S184>/CAN Unpack' */
extern real_T MCFR_ActualTorque;       /* '<S161>/CAN Unpack' */
extern real_T MCFR_ActualVelocity;     /* '<S161>/CAN Unpack' */
extern real_T MCFR_DCVoltage;          /* '<S161>/CAN Unpack' */
extern real_T MCFR_bDCOn;              /* '<S161>/CAN Unpack' */
extern real_T MCFR_bError;             /* '<S161>/CAN Unpack' */
extern real_T MCFR_bInverterOn;        /* '<S161>/CAN Unpack' */
extern real_T MCFR_bQuitInverterOn;    /* '<S161>/CAN Unpack' */
extern real_T MCFR_bSystemReady;       /* '<S161>/CAN Unpack' */
extern real_T MCFR_TempIGBT;           /* '<S172>/CAN Unpack' */
extern real_T MCFR_TempInverter;       /* '<S172>/CAN Unpack' */
extern real_T MCFR_TempMotor;          /* '<S172>/CAN Unpack' */
extern real_T MCFR_ErrorInfo;          /* '<S170>/CAN Unpack' */
extern real_T MCFL_ActualTorque;       /* '<S142>/CAN Unpack' */
extern real_T MCFL_ActualVelocity;     /* '<S142>/CAN Unpack' */
extern real_T MCFL_DCVoltage;          /* '<S142>/CAN Unpack' */
extern real_T MCFL_bDCOn;              /* '<S142>/CAN Unpack' */
extern real_T MCFL_bError;             /* '<S142>/CAN Unpack' */
extern real_T MCFL_bInverterOn;        /* '<S142>/CAN Unpack' */
extern real_T MCFL_bQuitDCOn;          /* '<S142>/CAN Unpack' */
extern real_T MCFL_bQuitInverterOn;    /* '<S142>/CAN Unpack' */
extern real_T MCFL_bSystemReady;       /* '<S142>/CAN Unpack' */
extern real_T MCFL_TempIGBT;           /* '<S154>/CAN Unpack' */
extern real_T MCFL_TempInverter;       /* '<S154>/CAN Unpack' */
extern real_T MCFL_TempMotor;          /* '<S154>/CAN Unpack' */
extern real_T MCFL_ErrorInfo;          /* '<S152>/CAN Unpack' */
extern real_T StrWhlAngAliveRollCnt;   /* '<S203>/CAN Unpack1' */
extern real_T StrWhlAng;               /* '<S203>/CAN Unpack1' */
extern real_T StrWhlAngV;              /* '<S203>/CAN Unpack1' */
extern real_T ABS_WS_FL;               /* '<S132>/CAN Unpack1' */
extern real_T ABS_WS_FR;               /* '<S132>/CAN Unpack1' */
extern real_T ABS_WS_RL;               /* '<S132>/CAN Unpack1' */
extern real_T ABS_WS_RR;               /* '<S132>/CAN Unpack1' */
extern real_T IMU_Ay_Value;            /* '<S198>/CAN Unpack' */
extern real_T IMU_Ax_Value;            /* '<S198>/CAN Unpack' */
extern real_T IMU_Yaw_Value;           /* '<S198>/CAN Unpack' */
extern real_T EMRAX_Trq_CUT;           /*  */
extern real_T AMK_Trq_CUT;             /*  */
extern uint32_T Acc_vol2;              /* '<S217>/Add3' */
extern uint32_T Acc_vol;               /* '<S217>/Add2' */
extern uint32_T Acc_POS;               /* '<S217>/1-D Lookup Table4' */
extern uint32_T Acc_POS2;              /* '<S217>/1-D Lookup Table3' */
extern real32_T VehVxEst_mps;          /* '<S345>/Add' */
extern real32_T PwrALL;                /* '<S28>/Gain3' */
extern real32_T EmraxTrqR_cmd;         /* '<S7>/Saturation1' */
extern real32_T AMKTrqFR_cmd;          /* '<S7>/Saturation3' */
extern real32_T AMKTrqFL_cmd;          /* '<S7>/Saturation4' */
extern uint16_T F_BrkPrs;              /* '<S217>/1-D Lookup Table1' */
extern uint16_T Acc1;                  /* '<S125>/Acc3' */
extern uint16_T Acc2;                  /* '<S125>/Acc4' */
extern uint16_T Brk1;                  /* '<S125>/Brk1' */
extern uint16_T Brk2;                  /* '<S125>/Brk2' */
extern boolean_T STATEDISPLAY;         /* '<S358>/Switch1' */
extern boolean_T HVSWITCH;             /* '<S358>/Chart' */
extern boolean_T TSAL_SW_IN;           /* '<S130>/SwitchInput2' */
extern boolean_T HV_voltValid;         /* '<S207>/Compare' */
extern boolean_T Brk;                  /* '<S115>/Compare' */
extern boolean_T ACC_Release;          /* '<S116>/Compare' */
extern boolean_T beeper_state;         /* '<S113>/Chart2' */
extern boolean_T MCFL_DCOn_setpoints;  /* '<S113>/Chart2' */
extern boolean_T MCFR_DCEnable;        /* '<S113>/Chart2' */
extern boolean_T MCFR_InverterOn;      /* '<S113>/Chart2' */
extern boolean_T TrqR_cmd_raw;         /* '<S7>/Logical Operator1' */
extern boolean_T TroqueOn;             /* '<S7>/Logical Operator6' */
extern boolean_T Trq_CUT_final;        /* '<S7>/Logical Operator4' */

/* External function called from main */
extern void VehCtrlMdel241025_2018b_amkspdlimit_SetEventsForThisBaseStep
  (boolean_T *eventFlags);

/* Model entry point functions */
extern void VehCtrlMdel241025_2018b_amkspdlimit_SetEventsForThisBaseStep
  (boolean_T *eventFlags);
extern void VehCtrlMdel241025_2018b_amkspdlimit_initialize(void);
extern void VehCtrlMdel241025_2018b_amkspdlimit_step(int_T tid);
extern uint8_T ECUCoderModelBaseCounter;
extern uint32_t IntcIsrVectorTable[];
extern uint8_T AfterRunFlags[2];
extern SSD_CONFIG ssdConfig;
extern void ISR_PIT_CH3(void);

/* Real-time Model object */
extern RT_MODEL_VehCtrlMdel241025_20_T *const VehCtrlMdel241025_2018b_amks_M;

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
 * Block '<S79>/Data Type Duplicate' : Unused code path elimination
 * Block '<S79>/Data Type Propagation' : Unused code path elimination
 * Block '<S80>/Data Type Duplicate' : Unused code path elimination
 * Block '<S80>/Data Type Propagation' : Unused code path elimination
 * Block '<S81>/Data Type Duplicate' : Unused code path elimination
 * Block '<S81>/Data Type Propagation' : Unused code path elimination
 * Block '<S88>/Data Type Duplicate' : Unused code path elimination
 * Block '<S88>/Data Type Propagation' : Unused code path elimination
 * Block '<S89>/Data Type Duplicate' : Unused code path elimination
 * Block '<S89>/Data Type Propagation' : Unused code path elimination
 * Block '<S90>/Data Type Duplicate' : Unused code path elimination
 * Block '<S90>/Data Type Propagation' : Unused code path elimination
 * Block '<S99>/Data Type Duplicate' : Unused code path elimination
 * Block '<S99>/Data Type Propagation' : Unused code path elimination
 * Block '<S100>/Data Type Duplicate' : Unused code path elimination
 * Block '<S100>/Data Type Propagation' : Unused code path elimination
 * Block '<S101>/Data Type Duplicate' : Unused code path elimination
 * Block '<S101>/Data Type Propagation' : Unused code path elimination
 * Block '<S113>/Switch5' : Unused code path elimination
 * Block '<S181>/CAN Unpack1' : Unused code path elimination
 * Block '<S217>/1-D Lookup Table2' : Unused code path elimination
 * Block '<S217>/Abs1' : Unused code path elimination
 * Block '<S217>/Add4' : Unused code path elimination
 * Block '<S234>/Compare' : Unused code path elimination
 * Block '<S234>/Constant' : Unused code path elimination
 * Block '<S243>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S249>/Data Type Duplicate' : Unused code path elimination
 * Block '<S249>/Data Type Propagation' : Unused code path elimination
 * Block '<S244>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S250>/Data Type Duplicate' : Unused code path elimination
 * Block '<S250>/Data Type Propagation' : Unused code path elimination
 * Block '<S245>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S251>/Data Type Duplicate' : Unused code path elimination
 * Block '<S251>/Data Type Propagation' : Unused code path elimination
 * Block '<S246>/Data Type Duplicate' : Unused code path elimination
 * Block '<S246>/Data Type Propagation' : Unused code path elimination
 * Block '<S247>/Data Type Duplicate' : Unused code path elimination
 * Block '<S247>/Data Type Propagation' : Unused code path elimination
 * Block '<S248>/Data Type Duplicate' : Unused code path elimination
 * Block '<S248>/Data Type Propagation' : Unused code path elimination
 * Block '<S253>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S256>/Data Type Duplicate' : Unused code path elimination
 * Block '<S256>/Data Type Propagation' : Unused code path elimination
 * Block '<S254>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S257>/Data Type Duplicate' : Unused code path elimination
 * Block '<S257>/Data Type Propagation' : Unused code path elimination
 * Block '<S273>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S281>/Data Type Duplicate' : Unused code path elimination
 * Block '<S281>/Data Type Propagation' : Unused code path elimination
 * Block '<S274>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S282>/Data Type Duplicate' : Unused code path elimination
 * Block '<S282>/Data Type Propagation' : Unused code path elimination
 * Block '<S275>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S283>/Data Type Duplicate' : Unused code path elimination
 * Block '<S283>/Data Type Propagation' : Unused code path elimination
 * Block '<S276>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S284>/Data Type Duplicate' : Unused code path elimination
 * Block '<S284>/Data Type Propagation' : Unused code path elimination
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
 * Block '<S327>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S329>/Data Type Duplicate' : Unused code path elimination
 * Block '<S329>/Data Type Propagation' : Unused code path elimination
 * Block '<S328>/Data Type Duplicate' : Unused code path elimination
 * Block '<S328>/Data Type Propagation' : Unused code path elimination
 * Block '<S330>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S332>/Data Type Duplicate' : Unused code path elimination
 * Block '<S332>/Data Type Propagation' : Unused code path elimination
 * Block '<S331>/Data Type Duplicate' : Unused code path elimination
 * Block '<S331>/Data Type Propagation' : Unused code path elimination
 * Block '<S333>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S335>/Data Type Duplicate' : Unused code path elimination
 * Block '<S335>/Data Type Propagation' : Unused code path elimination
 * Block '<S334>/Data Type Duplicate' : Unused code path elimination
 * Block '<S334>/Data Type Propagation' : Unused code path elimination
 * Block '<S336>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S338>/Data Type Duplicate' : Unused code path elimination
 * Block '<S338>/Data Type Propagation' : Unused code path elimination
 * Block '<S337>/Data Type Duplicate' : Unused code path elimination
 * Block '<S337>/Data Type Propagation' : Unused code path elimination
 * Block '<S288>/Discrete-Time Integrator' : Unused code path elimination
 * Block '<S352>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S354>/Data Type Duplicate' : Unused code path elimination
 * Block '<S354>/Data Type Propagation' : Unused code path elimination
 * Block '<S353>/Data Type Duplicate' : Unused code path elimination
 * Block '<S353>/Data Type Propagation' : Unused code path elimination
 * Block '<S346>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S355>/Data Type Duplicate' : Unused code path elimination
 * Block '<S355>/Data Type Propagation' : Unused code path elimination
 * Block '<S347>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S356>/Data Type Duplicate' : Unused code path elimination
 * Block '<S356>/Data Type Propagation' : Unused code path elimination
 * Block '<S348>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S357>/Data Type Duplicate' : Unused code path elimination
 * Block '<S357>/Data Type Propagation' : Unused code path elimination
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
 * Block '<S71>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S8>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S8>/Cast To Boolean1' : Eliminate redundant data type conversion
 * Block '<S8>/Cast To Boolean2' : Eliminate redundant data type conversion
 * Block '<S8>/Cast To Boolean3' : Eliminate redundant data type conversion
 * Block '<S217>/Abs' : Eliminated since data is unsigned
 * Block '<S216>/Cast To Double6' : Eliminate redundant data type conversion
 * Block '<S216>/Cast To Double7' : Eliminate redundant data type conversion
 * Block '<S285>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S285>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S287>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S287>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S287>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S287>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S345>/Gain1' : Eliminated nontunable gain of 1
 * Block '<S358>/Cast To Boolean1' : Eliminate redundant data type conversion
 * Block '<S361>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S362>/Cast To Single' : Eliminate redundant data type conversion
 * Block '<S362>/Cast To Single2' : Eliminate redundant data type conversion
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
 * '<Root>' : 'VehCtrlMdel241025_2018b_amkspdlimit'
 * '<S1>'   : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL'
 * '<S2>'   : 'VehCtrlMdel241025_2018b_amkspdlimit/Drive_Ready'
 * '<S3>'   : 'VehCtrlMdel241025_2018b_amkspdlimit/Input'
 * '<S4>'   : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing'
 * '<S5>'   : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT'
 * '<S6>'   : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting'
 * '<S7>'   : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq'
 * '<S8>'   : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct'
 * '<S9>'   : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Chart'
 * '<S10>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC'
 * '<S11>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule'
 * '<S12>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule1'
 * '<S13>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule2'
 * '<S14>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule3'
 * '<S15>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule4'
 * '<S16>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule5'
 * '<S17>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule6'
 * '<S18>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/MeaModule7'
 * '<S19>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic1'
 * '<S20>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic2'
 * '<S21>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic3'
 * '<S22>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Saturation Dynamic1'
 * '<S23>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Saturation Dynamic2'
 * '<S24>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Saturation Dynamic3'
 * '<S25>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Saturation Dynamic4'
 * '<S26>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Saturation Dynamic5'
 * '<S27>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Saturation Dynamic6'
 * '<S28>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem'
 * '<S29>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem1'
 * '<S30>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL'
 * '<S31>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR'
 * '<S32>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R'
 * '<S33>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant'
 * '<S34>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant1'
 * '<S35>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant2'
 * '<S36>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant3'
 * '<S37>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant4'
 * '<S38>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Compare To Constant5'
 * '<S39>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/MATLAB Function'
 * '<S40>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/MATLAB Function1'
 * '<S41>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic'
 * '<S42>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic1'
 * '<S43>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic2'
 * '<S44>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic3'
 * '<S45>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic4'
 * '<S46>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic5'
 * '<S47>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic6'
 * '<S48>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic'
 * '<S49>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic1'
 * '<S50>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic2'
 * '<S51>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic3'
 * '<S52>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic4'
 * '<S53>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic5'
 * '<S54>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic6'
 * '<S55>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Saturation Dynamic7'
 * '<S56>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Wtarget'
 * '<S57>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/载荷转移'
 * '<S58>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S59>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S60>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S61>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S62>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic4/Saturation Dynamic'
 * '<S63>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic5/Saturation Dynamic'
 * '<S64>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/DYC/Rate Limiter Dynamic6/Saturation Dynamic'
 * '<S65>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S66>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S67>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S68>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem/MeaModule4'
 * '<S69>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem/Subsystem1'
 * '<S70>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem/Subsystem1/Edge Detector'
 * '<S71>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem/Subsystem1/Edge Detector/Model'
 * '<S72>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem/Subsystem1/Edge Detector/Model/NEGATIVE Edge'
 * '<S73>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/Subsystem/Subsystem1/Edge Detector/Model/POSITIVE Edge'
 * '<S74>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Compare To Constant'
 * '<S75>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Compare To Constant2'
 * '<S76>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Detect Rise Positive'
 * '<S77>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Latch_on'
 * '<S78>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/RisingTimer'
 * '<S79>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic'
 * '<S80>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic1'
 * '<S81>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Saturation Dynamic2'
 * '<S82>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FL/Detect Rise Positive/Positive'
 * '<S83>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Compare To Constant'
 * '<S84>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Compare To Constant2'
 * '<S85>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Detect Rise Positive'
 * '<S86>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Latch_on'
 * '<S87>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/RisingTimer'
 * '<S88>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic'
 * '<S89>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic1'
 * '<S90>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Saturation Dynamic2'
 * '<S91>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_FR/Detect Rise Positive/Positive'
 * '<S92>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant'
 * '<S93>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant1'
 * '<S94>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Constant2'
 * '<S95>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Compare To Zero'
 * '<S96>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Detect Rise Positive'
 * '<S97>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Latch_on'
 * '<S98>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/RisingTimer'
 * '<S99>'  : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic'
 * '<S100>' : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic1'
 * '<S101>' : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Saturation Dynamic2'
 * '<S102>' : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/MoTrqReq/TCS_PI+FF_Ctrl_R/Detect Rise Positive/Positive'
 * '<S103>' : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant'
 * '<S104>' : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant1'
 * '<S105>' : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant2'
 * '<S106>' : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant3'
 * '<S107>' : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant4'
 * '<S108>' : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant5'
 * '<S109>' : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant6'
 * '<S110>' : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Compare To Constant7'
 * '<S111>' : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Timer1'
 * '<S112>' : 'VehCtrlMdel241025_2018b_amkspdlimit/CTRL/PwrTrainTempPrtct/Timer2'
 * '<S113>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Drive_Ready/Subsystem'
 * '<S114>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Drive_Ready/Subsystem/Chart2'
 * '<S115>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Drive_Ready/Subsystem/Compare To Constant'
 * '<S116>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Drive_Ready/Subsystem/Compare To Constant1'
 * '<S117>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule'
 * '<S118>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule1'
 * '<S119>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule2'
 * '<S120>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule3'
 * '<S121>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule4'
 * '<S122>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Drive_Ready/Subsystem/MeaModule5'
 * '<S123>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/ABS_Receive'
 * '<S124>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive'
 * '<S125>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AccBrk_BUS'
 * '<S126>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/BMS_Recive'
 * '<S127>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE'
 * '<S128>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/IMU_Recieve'
 * '<S129>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/StrSnis_Receive'
 * '<S130>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/key'
 * '<S131>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state'
 * '<S132>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state'
 * '<S133>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule1'
 * '<S134>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule2'
 * '<S135>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule3'
 * '<S136>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/ABS_Receive/ABS_BUS_state/IMU_state/MeaModule4'
 * '<S137>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU'
 * '<S138>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU'
 * '<S139>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state'
 * '<S140>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1'
 * '<S141>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2'
 * '<S142>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state'
 * '<S143>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule1'
 * '<S144>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule2'
 * '<S145>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule3'
 * '<S146>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule4'
 * '<S147>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule5'
 * '<S148>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule6'
 * '<S149>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule7'
 * '<S150>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule8'
 * '<S151>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state/MCU_state/MeaModule9'
 * '<S152>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state'
 * '<S153>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state1/MCU_state/MeaModule1'
 * '<S154>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state'
 * '<S155>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule1'
 * '<S156>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule2'
 * '<S157>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FLAMKMCU/AMKMCU_state2/MCU_state/MeaModule3'
 * '<S158>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state'
 * '<S159>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1'
 * '<S160>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2'
 * '<S161>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state'
 * '<S162>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule1'
 * '<S163>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule2'
 * '<S164>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule3'
 * '<S165>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule4'
 * '<S166>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule5'
 * '<S167>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule6'
 * '<S168>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule7'
 * '<S169>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state/MCU_state/MeaModule8'
 * '<S170>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state'
 * '<S171>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state1/MCU_state/MeaModule1'
 * '<S172>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state'
 * '<S173>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule1'
 * '<S174>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule2'
 * '<S175>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AMKMCU_Receive/FRAMKMCU/AMKMCU_state2/MCU_state/MeaModule3'
 * '<S176>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule1'
 * '<S177>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule2'
 * '<S178>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule3'
 * '<S179>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/AccBrk_BUS/MeaModule4'
 * '<S180>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/BMS_Recive/ABS_BUS_state'
 * '<S181>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/BMS_Recive/ABS_BUS_state/IMU_state'
 * '<S182>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr'
 * '<S183>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state'
 * '<S184>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1'
 * '<S185>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule1'
 * '<S186>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule2'
 * '<S187>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule3'
 * '<S188>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule4'
 * '<S189>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule5'
 * '<S190>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state'
 * '<S191>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule'
 * '<S192>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule1'
 * '<S193>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule2'
 * '<S194>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule3'
 * '<S195>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule5'
 * '<S196>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/EMRAXMCU_RECIEVE/MCU_state/MCU_state/MeaModule6'
 * '<S197>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state'
 * '<S198>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state'
 * '<S199>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule2'
 * '<S200>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule3'
 * '<S201>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/IMU_Recieve/IMU_state/MCU_state/MeaModule4'
 * '<S202>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state'
 * '<S203>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state'
 * '<S204>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule1'
 * '<S205>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule2'
 * '<S206>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/StrSnis_Receive/StrWhSnis_state/IMU_state/MeaModule3'
 * '<S207>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/key/Compare To Constant'
 * '<S208>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/key/MeaModule'
 * '<S209>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/key/MeaModule1'
 * '<S210>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/key/MeaModule2'
 * '<S211>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/key/MeaModule3'
 * '<S212>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/key/Timer'
 * '<S213>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/key/Timer1'
 * '<S214>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input/key/Timer2'
 * '<S215>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem'
 * '<S216>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem'
 * '<S217>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal'
 * '<S218>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU'
 * '<S219>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS'
 * '<S220>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem'
 * '<S221>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem1'
 * '<S222>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps'
 * '<S223>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant'
 * '<S224>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant1'
 * '<S225>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant10'
 * '<S226>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant11'
 * '<S227>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant2'
 * '<S228>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant3'
 * '<S229>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant4'
 * '<S230>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant5'
 * '<S231>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant6'
 * '<S232>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant7'
 * '<S233>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant8'
 * '<S234>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Compare To Constant9'
 * '<S235>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule1'
 * '<S236>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule2'
 * '<S237>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule3'
 * '<S238>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule5'
 * '<S239>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule7'
 * '<S240>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/MeaModule8'
 * '<S241>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/AccBrkPedal/Timer'
 * '<S242>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem'
 * '<S243>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic'
 * '<S244>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1'
 * '<S245>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2'
 * '<S246>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic'
 * '<S247>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic1'
 * '<S248>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Saturation Dynamic2'
 * '<S249>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S250>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S251>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/IMU/Subsystem/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S252>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Compare To Constant'
 * '<S253>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic'
 * '<S254>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1'
 * '<S255>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Timer'
 * '<S256>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S257>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/SWAS/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S258>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem/MeaModule'
 * '<S259>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem/MeaModule1'
 * '<S260>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem/MeaModule2'
 * '<S261>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem1/Chart'
 * '<S262>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem1/Detect Rise Positive1'
 * '<S263>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem1/Timer'
 * '<S264>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/Subsystem1/Detect Rise Positive1/Positive'
 * '<S265>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant'
 * '<S266>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant1'
 * '<S267>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant2'
 * '<S268>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant3'
 * '<S269>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant4'
 * '<S270>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant5'
 * '<S271>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant6'
 * '<S272>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Compare To Constant7'
 * '<S273>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic'
 * '<S274>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1'
 * '<S275>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2'
 * '<S276>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3'
 * '<S277>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer'
 * '<S278>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer1'
 * '<S279>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer2'
 * '<S280>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Timer3'
 * '<S281>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S282>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S283>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S284>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Function-Call Subsystem/WhlSpdii_mps/Rate Limiter Dynamic3/Saturation Dynamic'
 * '<S285>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment '
 * '<S286>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/MeaModule'
 * '<S287>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment'
 * '<S288>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect'
 * '<S289>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/坐标系转换'
 * '<S290>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment /Compare To Constant'
 * '<S291>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/IntglJudgment /Compare To Constant1'
 * '<S292>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer'
 * '<S293>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer1'
 * '<S294>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer2'
 * '<S295>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/Timer3'
 * '<S296>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断'
 * '<S297>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断'
 * '<S298>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断'
 * '<S299>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant'
 * '<S300>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant1'
 * '<S301>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant2'
 * '<S302>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Compare To Constant3'
 * '<S303>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter'
 * '<S304>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1'
 * '<S305>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2'
 * '<S306>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3'
 * '<S307>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic'
 * '<S308>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Saturation Dynamic'
 * '<S309>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S310>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic'
 * '<S311>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Saturation Dynamic'
 * '<S312>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S313>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic'
 * '<S314>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Saturation Dynamic'
 * '<S315>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S316>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic'
 * '<S317>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Saturation Dynamic'
 * '<S318>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S319>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant'
 * '<S320>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant1'
 * '<S321>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant2'
 * '<S322>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Compare To Constant3'
 * '<S323>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter'
 * '<S324>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1'
 * '<S325>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2'
 * '<S326>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3'
 * '<S327>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic'
 * '<S328>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Saturation Dynamic'
 * '<S329>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S330>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic'
 * '<S331>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Saturation Dynamic'
 * '<S332>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S333>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic'
 * '<S334>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Saturation Dynamic'
 * '<S335>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S336>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic'
 * '<S337>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Saturation Dynamic'
 * '<S338>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/加速度差的微分的判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S339>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant'
 * '<S340>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant1'
 * '<S341>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant2'
 * '<S342>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSlpageJudgment/速差判断/Compare To Constant3'
 * '<S343>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant'
 * '<S344>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Compare To Constant1'
 * '<S345>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter'
 * '<S346>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic'
 * '<S347>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1'
 * '<S348>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2'
 * '<S349>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Subsystem'
 * '<S350>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Timer1'
 * '<S351>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Timer2'
 * '<S352>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic'
 * '<S353>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Saturation Dynamic'
 * '<S354>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S355>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S356>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S357>' : 'VehCtrlMdel241025_2018b_amkspdlimit/Input_Processing/Subsystem/WhlSpdSelect/Rate Limiter Dynamic2/Saturation Dynamic'
 * '<S358>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/Beeper'
 * '<S359>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/VCU2AMKMCUFL'
 * '<S360>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/VCU2AMKMCUFR'
 * '<S361>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU'
 * '<S362>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/WP_OUTPUT'
 * '<S363>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/Beeper/Chart'
 * '<S364>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/Beeper/Compare To Constant2'
 * '<S365>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/Beeper/Detect Rise Positive'
 * '<S366>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/Beeper/Enabled Subsystem1'
 * '<S367>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/Beeper/MeaModule'
 * '<S368>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/Beeper/MeaModule1'
 * '<S369>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/Beeper/MeaModule2'
 * '<S370>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/Beeper/Subsystem'
 * '<S371>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/Beeper/Timer2'
 * '<S372>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/Beeper/Timer3'
 * '<S373>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/Beeper/Detect Rise Positive/Positive'
 * '<S374>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/Beeper/Enabled Subsystem1/Chart'
 * '<S375>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/Beeper/Subsystem/Chart1'
 * '<S376>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU/MeaModule1'
 * '<S377>' : 'VehCtrlMdel241025_2018b_amkspdlimit/OUTPUT/VCU2EmraxMCU/MeaModule2'
 * '<S378>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/BL'
 * '<S379>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/DAQ'
 * '<S380>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/EEPROM'
 * '<S381>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/Polling'
 * '<S382>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem'
 * '<S383>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem'
 * '<S384>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem'
 * '<S385>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com0'
 * '<S386>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com1'
 * '<S387>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com2'
 * '<S388>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem'
 * '<S389>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem1'
 * '<S390>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/DAQ/daq100ms'
 * '<S391>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/DAQ/daq10ms'
 * '<S392>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/DAQ/daq500ms'
 * '<S393>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/DAQ/daq50ms'
 * '<S394>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/DAQ/daq5ms'
 * '<S395>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/EEPROM/EEPROMOperation'
 * '<S396>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/Polling/CCPBackground'
 * '<S397>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/Polling/CCPReceive'
 * '<S398>' : 'VehCtrlMdel241025_2018b_amkspdlimit/RapidECUSetting/Polling/CCPReceive/Nothing'
 */
#endif                   /* RTW_HEADER_VehCtrlMdel241025_2018b_amkspdlimit_h_ */

/* File trailer for ECUCoder generated file VehCtrlMdel241025_2018b_amkspdlimit.h.
 *
 * [EOF]
 */
