/*
 * Code generated for Simulink model RP20231104WLR_231105.
 *
 * FILE    : RP20231104WLR_231105.h
 *
 * VERSION : 1.1
 *
 * DATE    : Sat Sep 21 10:42:01 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#ifndef RTW_HEADER_RP20231104WLR_231105_h_
#define RTW_HEADER_RP20231104WLR_231105_h_
#include <math.h>
#include "MPC5744P.h"
#include "Std_Types.h"
#include "can.h"
#include "flash.h"
#include "crc.h"
#ifndef RP20231104WLR_231105_COMMON_INCLUDES_
# define RP20231104WLR_231105_COMMON_INCLUDES_
#include <string.h>
#include <math.h>
#include "rtwtypes.h"
#include "can_message.h"
#endif                               /* RP20231104WLR_231105_COMMON_INCLUDES_ */

#include "RP20231104WLR_231105_types.h"
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

/* Block signals (default storage) */
typedef struct {
  CAN_DATATYPE CANPack1;               /* '<S69>/CAN Pack1' */
  CAN_DATATYPE CANPack;                /* '<S68>/CAN Pack' */
  CAN_DATATYPE CANPack1_d;             /* '<S68>/CAN Pack1' */
  CAN_DATATYPE CANUnPackMessage4;      /* '<S58>/CANUnPackMessage4' */
  CAN_DATATYPE CANUnPackMessage4_b;    /* '<S52>/CANUnPackMessage4' */
  real_T low_VOL;                      /* '<S58>/CAN Unpack' */
  real_T MCU_Temp_error;               /* '<S58>/CAN Unpack' */
  real_T Mode;                         /* '<S58>/CAN Unpack' */
  real_T motorTemp_error;              /* '<S58>/CAN Unpack' */
  real_T overCurrent;                  /* '<S58>/CAN Unpack' */
  real_T overpower;                    /* '<S58>/CAN Unpack' */
  real_T overvol;                      /* '<S58>/CAN Unpack' */
  real_T Precharge;                    /* '<S58>/CAN Unpack' */
  real_T Reslove_error;                /* '<S58>/CAN Unpack' */
  real_T TCS_TrqRequestFinal_Nm;       /* '<S14>/Switch2' */
  uint32_T CANReceive_o3;              /* '<S93>/CANReceive' */
  uint32_T CANReceive_o3_c;            /* '<S78>/CANReceive' */
  uint32_T CANReceive1_o3;             /* '<S25>/CANReceive1' */
  uint32_T CANReceive3_o3;             /* '<S25>/CANReceive3' */
  real32_T DataTypeConversion1;        /* '<S69>/Data Type Conversion1' */
  real32_T voltage;                    /* '<S52>/CAN Unpack' */
  real32_T Brk_R;                      /* '<S23>/Data Type Conversion3' */
  real32_T Brk_F;                      /* '<S23>/Data Type Conversion' */
  real32_T Acc_POS;                    /* '<S23>/Divide' */
  int32_T DataTypeConversion2;         /* '<S69>/Data Type Conversion2' */
  uint16_T CastToBoolean4;             /* '<S69>/Cast To Boolean4' */
  uint16_T CastToBoolean6;             /* '<S69>/Cast To Boolean6' */
  uint16_T FrequencyRead;              /* '<S24>/FrequencyRead' */
  uint16_T Acc1;                       /* '<S23>/Acc1' */
  uint16_T Brk_F_l;                    /* '<S23>/Brk_F' */
  uint16_T Brk_R_a;                    /* '<S23>/Brk_R' */
  uint16_T Acc2;                       /* '<S23>/Acc2' */
  uint16_T Water_pump;                 /* '<S7>/Data Type Conversion3' */
  uint8_T CANReceive_o2;               /* '<S93>/CANReceive' */
  uint8_T CANReceive_o4[8];            /* '<S93>/CANReceive' */
  uint8_T CANReceive_o5;               /* '<S93>/CANReceive' */
  uint8_T CANReceive_o2_o;             /* '<S78>/CANReceive' */
  uint8_T CANReceive_o4_p[8];          /* '<S78>/CANReceive' */
  uint8_T CANReceive_o5_k;             /* '<S78>/CANReceive' */
  uint8_T CANTransmit;                 /* '<S85>/CANTransmit' */
  uint8_T CANPackMessage[8];           /* '<S69>/CANPackMessage' */
  uint8_T CANTransmit_d;               /* '<S69>/CANTransmit' */
  uint8_T CANPackMessage_o[8];         /* '<S68>/CANPackMessage' */
  uint8_T CANPackMessage1[8];          /* '<S68>/CANPackMessage1' */
  uint8_T CANTransmit_p;               /* '<S68>/CANTransmit' */
  uint8_T CANTransmit1;                /* '<S68>/CANTransmit1' */
  uint8_T CANReceive1_o2;              /* '<S25>/CANReceive1' */
  uint8_T CANReceive1_o4[8];           /* '<S25>/CANReceive1' */
  uint8_T CANReceive1_o5;              /* '<S25>/CANReceive1' */
  uint8_T CANReceive3_o2;              /* '<S25>/CANReceive3' */
  uint8_T CANReceive3_o4[8];           /* '<S25>/CANReceive3' */
  uint8_T CANReceive3_o5;              /* '<S25>/CANReceive3' */
  boolean_T LVMS;                      /* '<S26>/KL15Monitor' */
  boolean_T Drive_ready;               /* '<S26>/SwitchInput' */
  boolean_T LogicalOperator5;          /* '<S17>/Logical Operator5' */
} B_RP20231104WLR_231105_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay1_DSTATE;            /* '<S14>/Unit Delay1' */
  real_T UnitDelay4_DSTATE;            /* '<S14>/Unit Delay4' */
  real_T UnitDelay_DSTATE;             /* '<S14>/Unit Delay' */
  real_T UnitDelay2_DSTATE;            /* '<S14>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_c;         /* '<S38>/Unit Delay' */
  real32_T UnitDelay_DSTATE_p;         /* '<S39>/Unit Delay' */
  real32_T DelayInput2_DSTATE;         /* '<S12>/Delay Input2' */
  real32_T UnitDelay1_DSTATE_d;        /* '<S18>/Unit Delay1' */
  uint32_T previousTicks;              /* '<S95>/Chart1' */
  uint32_T MoTrqReq_PREV_T;            /* '<S1>/MoTrqReq' */
  int_T CANPack1_ModeSignalID;         /* '<S69>/CAN Pack1' */
  int_T CANPack_ModeSignalID;          /* '<S68>/CAN Pack' */
  int_T CANPack1_ModeSignalID_d;       /* '<S68>/CAN Pack1' */
  int_T CANUnpack_ModeSignalID;        /* '<S58>/CAN Unpack' */
  int_T CANUnpack_StatusPortID;        /* '<S58>/CAN Unpack' */
  int_T CANUnpack_ModeSignalID_n;      /* '<S52>/CAN Unpack' */
  int_T CANUnpack_StatusPortID_i;      /* '<S52>/CAN Unpack' */
  struct {
    uint_T is_VehStat:3;               /* '<S95>/Chart1' */
    uint_T is_BeeperStat:2;            /* '<S95>/Chart1' */
    uint_T is_active_c1_RP20231104WLR_2311:1;/* '<S95>/Chart1' */
    uint_T is_active_VehStat:1;        /* '<S95>/Chart1' */
    uint_T is_active_Output:1;         /* '<S95>/Chart1' */
    uint_T is_active_BeeperStat:1;     /* '<S95>/Chart1' */
  } bitsForTID3;

  uint16_T UnitDelay_DSTATE_f;         /* '<S23>/Unit Delay' */
  uint16_T UnitDelay1_DSTATE_n;        /* '<S23>/Unit Delay1' */
  boolean_T UnitDelay1_DSTATE_j;       /* '<S17>/Unit Delay1' */
  boolean_T UnitDelay3_DSTATE;         /* '<S14>/Unit Delay3' */
  boolean_T DelayInput1_DSTATE;        /* '<S16>/Delay Input1' */
  uint8_T temporalCounter_i1;          /* '<S95>/Chart1' */
  boolean_T MoTrqReq_RESET_ELAPS_T;    /* '<S1>/MoTrqReq' */
} DW_RP20231104WLR_231105_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: [0.3,0.3,2.5,2.5]
   * Referenced by: '<S14>/VehSpd_SlipTarget_mps'
   */
  real_T VehSpd_SlipTarget_mps_tableData[4];

  /* Pooled Parameter (Expression: [0,3,25,30])
   * Referenced by:
   *   '<S14>/VehSpd_SlipTarget_mps'
   *   '<S14>/VehicleStableTarget_mps'
   *   '<S14>/VehicleStableTarget_mps1'
   */
  real_T pooled4[4];

  /* Pooled Parameter (Expression: [0.4,0.4,1.2,1.2])
   * Referenced by:
   *   '<S14>/VehicleStableTarget_mps'
   *   '<S14>/VehicleStableTarget_mps1'
   */
  real_T pooled5[4];

  /* Expression: single([1000,0]);
   * Referenced by: '<S8>/BrakeCompensateCoef'
   */
  real32_T BrakeCompensateCoef_tableData[2];

  /* Expression: single([550,2000]);
   * Referenced by: '<S8>/BrakeCompensateCoef'
   */
  real32_T BrakeCompensateCoef_bp01Data[2];

  /* Expression: single(reshape([0,20,35,50,79,100,120,125,130,130,170,0,20,35,50,79,100,120,125,160,160,170,0,20,35,50,79,100,120,125,160,160,180,0,20,35,50,79,105,120,130,160,160,180,0,20,35,50,85,105,125,135,160,160,180,0,20,35,50,85,105,125,140,170,170,210,0,20,30,50,85,110,125,145,160,160,200,0,18,30,50,90,110,135,155,160,160,200,0,18,25,50,95,110,135,155,160,160,200,0,18,25,40,100,110,135,155,160,160,200,0,17,25,40,110,115,135,155,160,160,200,0,17,25,40,110,115,135,155,160,160,200,0,16,20,40,110,125,135,155,160,160,190,0,16,20,40,100,125,130,155,160,160,180,0,15,20,40,90,120,130,155,160,160,180,0,15,20,40,90,110,120,135,142,142,152,0,15,20,40,80,90,100,110,120,120,133,0,14,20,40,72,80,90,100,105,105,118,0,14,20,38,70,76,82,88,94,95,106,0,14,18,35,65,70,74,82,88,91,100],11,20));
   * Referenced by: '<S8>/2-D Lookup Table'
   */
  real32_T uDLookupTable_tableData[220];

  /* Expression: single([0,10,20,30,40,50,60,70,80,90,100]);
   * Referenced by: '<S8>/2-D Lookup Table'
   */
  real32_T uDLookupTable_bp01Data[11];

  /* Expression: single([200,400,600,800,1000,1200,1400,1600,1800,2000,2200,2400,2600,2800,3000,3500,4000,4500,5000,5200]);
   * Referenced by: '<S8>/2-D Lookup Table'
   */
  real32_T uDLookupTable_bp02Data[20];

  /* Computed Parameter: MCU__spd_tableData
   * Referenced by: '<S7>/MCU__spd'
   */
  real32_T MCU__spd_tableData[14];

  /* Computed Parameter: MCU__spd_bp01Data
   * Referenced by: '<S7>/MCU__spd'
   */
  real32_T MCU__spd_bp01Data[14];

  /* Computed Parameter: moter__spd_tableData
   * Referenced by: '<S7>/moter__spd'
   */
  real32_T moter__spd_tableData[13];

  /* Computed Parameter: moter__spd_bp01Data
   * Referenced by: '<S7>/moter__spd'
   */
  real32_T moter__spd_bp01Data[13];

  /* Pooled Parameter (Expression: single([0,100]);)
   * Referenced by:
   *   '<S23>/1-D Lookup Table'
   *   '<S23>/1-D Lookup Table3'
   */
  real32_T pooled11[2];

  /* Expression: single([1330,1590]);
   * Referenced by: '<S23>/1-D Lookup Table'
   */
  real32_T uDLookupTable_bp01Data_o[2];

  /* Expression: single([2324,2600]);
   * Referenced by: '<S23>/1-D Lookup Table3'
   */
  real32_T uDLookupTable3_bp01Data[2];

  /* Pooled Parameter (Expression: single([0,10000]);)
   * Referenced by:
   *   '<S23>/1-D Lookup Table1'
   *   '<S23>/1-D Lookup Table2'
   */
  real32_T pooled14[2];

  /* Expression: single([530,4500]);
   * Referenced by: '<S23>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_bp01Data[2];

  /* Expression: single([500,4500]);
   * Referenced by: '<S23>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_bp01Data[2];

  /* Computed Parameter: uDLookupTable_maxIndex
   * Referenced by: '<S8>/2-D Lookup Table'
   */
  uint32_T uDLookupTable_maxIndex[2];
} ConstP_RP20231104WLR_231105_T;

/* Real-time Model Data Structure */
struct tag_RTM_RP20231104WLR_231105_T {
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
extern B_RP20231104WLR_231105_T RP20231104WLR_231105_B;

/* Block states (default storage) */
extern DW_RP20231104WLR_231105_T RP20231104WLR_231105_DW;

/* Constant parameters (default storage) */
extern const ConstP_RP20231104WLR_231105_T RP20231104WLR_231105_ConstP;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real_T Gear_Trs;                /* '<S69>/Switch2' */
extern real_T Mode_Trs;                /* '<S69>/Switch3' */
extern real_T L12V_error;              /* '<S58>/CAN Unpack' */
extern real_T alarm;                   /* '<S58>/CAN Unpack' */
extern real_T controller_ready;        /* '<S58>/CAN Unpack' */
extern real_T selfcheck;               /* '<S58>/CAN Unpack' */
extern real_T RPM;                     /* '<S58>/CAN Unpack' */
extern real_T trq;                     /* '<S58>/CAN Unpack' */
extern real_T Spd_R;                   /* '<S58>/rpm2kph' */
extern real_T motor_power;             /* '<S52>/Divide' */
extern real_T Wheel_spd_F;             /* '<S24>/Gain' */
extern real_T VCU_SpdCmd;              /* '<S8>/RPM_Saturation' */
extern uint32_T Brk_F_vol;             /* '<S38>/Data Type Conversion1' */
extern uint32_T Brk_R_vol;             /* '<S39>/Data Type Conversion1' */
extern uint32_T R_BrkPrs;              /* '<S23>/1-D Lookup Table2' */
extern uint32_T F_BrkPrs;              /* '<S23>/1-D Lookup Table1' */
extern uint32_T Acc_vol;               /* '<S23>/Add2' */
extern uint32_T Acc_vol2;              /* '<S23>/Add3' */
extern uint32_T Acc_POS1;              /* '<S23>/1-D Lookup Table' */
extern uint32_T Acc_POS2;              /* '<S23>/1-D Lookup Table3' */
extern real32_T AC_current;            /* '<S52>/CAN Unpack' */
extern real32_T DC_current;            /* '<S52>/CAN Unpack' */
extern real32_T MCU_Temp;              /* '<S52>/CAN Unpack' */
extern real32_T motor_Temp;            /* '<S52>/CAN Unpack' */
extern real32_T VCU_TrqCmd;            /* '<S8>/Saturation' */
extern boolean_T VehReady;             /* '<S95>/Chart1' */
extern boolean_T beeper_state;         /* '<S95>/Chart1' */
extern boolean_T ignition;             /* '<S26>/Logical Operator' */
extern boolean_T Trq_CUT;              /* '<S23>/Logical Operator4' */

/* External function called from main */
extern void RP20231104WLR_231105_SetEventsForThisBaseStep(boolean_T *eventFlags);

/* Model entry point functions */
extern void RP20231104WLR_231105_SetEventsForThisBaseStep(boolean_T *eventFlags);
extern void RP20231104WLR_231105_initialize(void);
extern void RP20231104WLR_231105_step(int_T tid);
extern uint8_T ECUCoderModelBaseCounter;
extern uint32_t IntcIsrVectorTable[];
extern uint8_T AfterRunFlags[2];
extern SSD_CONFIG ssdConfig;
extern void ISR_PIT_CH3(void);

/* Real-time Model object */
extern RT_MODEL_RP20231104WLR_231105_T *const RP20231104WLR_231105_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S12>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S15>/Data Type Duplicate' : Unused code path elimination
 * Block '<S15>/Data Type Propagation' : Unused code path elimination
 * Block '<S13>/Data Type Duplicate' : Unused code path elimination
 * Block '<S13>/Data Type Propagation' : Unused code path elimination
 * Block '<S19>/Data Type Duplicate' : Unused code path elimination
 * Block '<S19>/Data Type Propagation' : Unused code path elimination
 * Block '<S20>/Data Type Duplicate' : Unused code path elimination
 * Block '<S20>/Data Type Propagation' : Unused code path elimination
 * Block '<S21>/Data Type Duplicate' : Unused code path elimination
 * Block '<S21>/Data Type Propagation' : Unused code path elimination
 * Block '<S5>/Constant' : Unused code path elimination
 * Block '<S8>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S8>/Data Type Conversion4' : Eliminate redundant data type conversion
 * Block '<S8>/Data Type Conversion5' : Eliminate redundant data type conversion
 * Block '<S2>/Data Type  Conversion' : Eliminate redundant data type conversion
 * Block '<S95>/Data Type Conversion1' : Eliminate redundant data type conversion
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
 * '<Root>' : 'RP20231104WLR_231105'
 * '<S1>'   : 'RP20231104WLR_231105/Ctrl'
 * '<S2>'   : 'RP20231104WLR_231105/Init'
 * '<S3>'   : 'RP20231104WLR_231105/Input'
 * '<S4>'   : 'RP20231104WLR_231105/Output'
 * '<S5>'   : 'RP20231104WLR_231105/RapidECUSetting'
 * '<S6>'   : 'RP20231104WLR_231105/drive_ready'
 * '<S7>'   : 'RP20231104WLR_231105/Ctrl/FAN_WP_CTRL'
 * '<S8>'   : 'RP20231104WLR_231105/Ctrl/MoTrqReq'
 * '<S9>'   : 'RP20231104WLR_231105/Ctrl/MoTrqReq/MeaModule1'
 * '<S10>'  : 'RP20231104WLR_231105/Ctrl/MoTrqReq/MeaModule2'
 * '<S11>'  : 'RP20231104WLR_231105/Ctrl/MoTrqReq/MotorTrqLimit'
 * '<S12>'  : 'RP20231104WLR_231105/Ctrl/MoTrqReq/Rate Limiter Dynamic'
 * '<S13>'  : 'RP20231104WLR_231105/Ctrl/MoTrqReq/Saturation Dynamic'
 * '<S14>'  : 'RP20231104WLR_231105/Ctrl/MoTrqReq/TCS_PI+FF_Ctrl'
 * '<S15>'  : 'RP20231104WLR_231105/Ctrl/MoTrqReq/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S16>'  : 'RP20231104WLR_231105/Ctrl/MoTrqReq/TCS_PI+FF_Ctrl/Detect Rise Positive'
 * '<S17>'  : 'RP20231104WLR_231105/Ctrl/MoTrqReq/TCS_PI+FF_Ctrl/Latch_on'
 * '<S18>'  : 'RP20231104WLR_231105/Ctrl/MoTrqReq/TCS_PI+FF_Ctrl/RisingTimer'
 * '<S19>'  : 'RP20231104WLR_231105/Ctrl/MoTrqReq/TCS_PI+FF_Ctrl/Saturation Dynamic'
 * '<S20>'  : 'RP20231104WLR_231105/Ctrl/MoTrqReq/TCS_PI+FF_Ctrl/Saturation Dynamic1'
 * '<S21>'  : 'RP20231104WLR_231105/Ctrl/MoTrqReq/TCS_PI+FF_Ctrl/Saturation Dynamic2'
 * '<S22>'  : 'RP20231104WLR_231105/Ctrl/MoTrqReq/TCS_PI+FF_Ctrl/Detect Rise Positive/Positive'
 * '<S23>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS'
 * '<S24>'  : 'RP20231104WLR_231105/Input/Function-Call Subsystem1'
 * '<S25>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE'
 * '<S26>'  : 'RP20231104WLR_231105/Input/key'
 * '<S27>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/Compare To Constant'
 * '<S28>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/Compare To Constant1'
 * '<S29>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/Compare To Constant10'
 * '<S30>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/Compare To Constant2'
 * '<S31>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/Compare To Constant3'
 * '<S32>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/Compare To Constant4'
 * '<S33>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/Compare To Constant5'
 * '<S34>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/Compare To Constant6'
 * '<S35>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/Compare To Constant7'
 * '<S36>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/Compare To Constant8'
 * '<S37>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/Compare To Constant9'
 * '<S38>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/LowPass2'
 * '<S39>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/LowPass4'
 * '<S40>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/MeaModule'
 * '<S41>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/MeaModule1'
 * '<S42>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/MeaModule2'
 * '<S43>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/MeaModule3'
 * '<S44>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/MeaModule4'
 * '<S45>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/MeaModule5'
 * '<S46>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/MeaModule6'
 * '<S47>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/MeaModule7'
 * '<S48>'  : 'RP20231104WLR_231105/Input/AccBrk_BUS/MeaModule8'
 * '<S49>'  : 'RP20231104WLR_231105/Input/Function-Call Subsystem1/MeaModule1'
 * '<S50>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_pwr'
 * '<S51>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_state'
 * '<S52>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_pwr/MCU_VCUMeter1'
 * '<S53>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule'
 * '<S54>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule1'
 * '<S55>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule2'
 * '<S56>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule3'
 * '<S57>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_pwr/MCU_VCUMeter1/MeaModule4'
 * '<S58>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_state/MCU_state'
 * '<S59>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_state/MCU_state/MeaModule'
 * '<S60>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_state/MCU_state/MeaModule1'
 * '<S61>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_state/MCU_state/MeaModule2'
 * '<S62>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_state/MCU_state/MeaModule3'
 * '<S63>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_state/MCU_state/MeaModule4'
 * '<S64>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_state/MCU_state/MeaModule5'
 * '<S65>'  : 'RP20231104WLR_231105/Input/MCU_RECIEVE/MCU_state/MCU_state/MeaModule6'
 * '<S66>'  : 'RP20231104WLR_231105/Input/key/MeaModule'
 * '<S67>'  : 'RP20231104WLR_231105/Output/Beeper'
 * '<S68>'  : 'RP20231104WLR_231105/Output/VCU2CLUSTER'
 * '<S69>'  : 'RP20231104WLR_231105/Output/VCU2MCU'
 * '<S70>'  : 'RP20231104WLR_231105/Output/WP_OUTPUT'
 * '<S71>'  : 'RP20231104WLR_231105/Output/Beeper/MeaModule'
 * '<S72>'  : 'RP20231104WLR_231105/Output/VCU2MCU/MeaModule1'
 * '<S73>'  : 'RP20231104WLR_231105/Output/VCU2MCU/MeaModule2'
 * '<S74>'  : 'RP20231104WLR_231105/RapidECUSetting/BL'
 * '<S75>'  : 'RP20231104WLR_231105/RapidECUSetting/DAQ'
 * '<S76>'  : 'RP20231104WLR_231105/RapidECUSetting/EEPROM'
 * '<S77>'  : 'RP20231104WLR_231105/RapidECUSetting/Polling'
 * '<S78>'  : 'RP20231104WLR_231105/RapidECUSetting/BL/Function-Call Subsystem'
 * '<S79>'  : 'RP20231104WLR_231105/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem'
 * '<S80>'  : 'RP20231104WLR_231105/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem'
 * '<S81>'  : 'RP20231104WLR_231105/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com0'
 * '<S82>'  : 'RP20231104WLR_231105/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com1'
 * '<S83>'  : 'RP20231104WLR_231105/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/Com2'
 * '<S84>'  : 'RP20231104WLR_231105/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem'
 * '<S85>'  : 'RP20231104WLR_231105/RapidECUSetting/BL/Function-Call Subsystem/Function-Call Subsystem/Enabled Subsystem/If Action Subsystem1'
 * '<S86>'  : 'RP20231104WLR_231105/RapidECUSetting/DAQ/daq100ms'
 * '<S87>'  : 'RP20231104WLR_231105/RapidECUSetting/DAQ/daq10ms'
 * '<S88>'  : 'RP20231104WLR_231105/RapidECUSetting/DAQ/daq500ms'
 * '<S89>'  : 'RP20231104WLR_231105/RapidECUSetting/DAQ/daq50ms'
 * '<S90>'  : 'RP20231104WLR_231105/RapidECUSetting/DAQ/daq5ms'
 * '<S91>'  : 'RP20231104WLR_231105/RapidECUSetting/EEPROM/EEPROMOperation'
 * '<S92>'  : 'RP20231104WLR_231105/RapidECUSetting/Polling/CCPBackground'
 * '<S93>'  : 'RP20231104WLR_231105/RapidECUSetting/Polling/CCPReceive'
 * '<S94>'  : 'RP20231104WLR_231105/RapidECUSetting/Polling/CCPReceive/Nothing'
 * '<S95>'  : 'RP20231104WLR_231105/drive_ready/Drive_ready'
 * '<S96>'  : 'RP20231104WLR_231105/drive_ready/Drive_ready/Chart1'
 * '<S97>'  : 'RP20231104WLR_231105/drive_ready/Drive_ready/Compare To Constant'
 * '<S98>'  : 'RP20231104WLR_231105/drive_ready/Drive_ready/Compare To Constant2'
 * '<S99>'  : 'RP20231104WLR_231105/drive_ready/Drive_ready/MeaModule'
 */
#endif                                 /* RTW_HEADER_RP20231104WLR_231105_h_ */

/* File trailer for ECUCoder generated file RP20231104WLR_231105.h.
 *
 * [EOF]
 */
