/*
 * Code generated for Simulink model VehCtrlMdel240926_2018b_amkspdlimit.
 *
 * FILE    : VehCtrlMdel240926_2018b_amkspdlimit_types.h
 *
 * VERSION : 1.230
 *
 * DATE    : Sat Oct 12 03:28:34 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#ifndef RTW_HEADER_VehCtrlMdel240926_2018b_amkspdlimit_types_h_
#define RTW_HEADER_VehCtrlMdel240926_2018b_amkspdlimit_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_struct_yQsOshnkIdTklzMvw9YDE_
#define DEFINED_TYPEDEF_FOR_struct_yQsOshnkIdTklzMvw9YDE_

typedef struct {
  boolean_T Trq_CUT;
  real32_T TrqR_cmd;
  real32_T TrqFR_cmd;
  real32_T TrqFL_cmd;
  boolean_T TCSFL_flgActv;
  boolean_T TCSFR_flgActv;
  boolean_T TCSR_flgActv;
  real_T VCU_SpdCmd_Emrax;
} struct_yQsOshnkIdTklzMvw9YDE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_x0NdDYyyx9QGAp231ZZ3YD_
#define DEFINED_TYPEDEF_FOR_struct_x0NdDYyyx9QGAp231ZZ3YD_

typedef struct {
  real_T SingleAMKPowerLimit_KW;
  real_T allAMKMotorTrq_CUT;
  boolean_T aWaterPumpON;
  boolean_T bWaterPumpON;
  real_T SingleEmraxPowerLimit_KW;
  real_T EmraxMotorTrq_CUT;
  real32_T LoadOfbWaterPump;
} struct_x0NdDYyyx9QGAp231ZZ3YD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_41ZI6UPtvQp2dx0RbZz7gH_
#define DEFINED_TYPEDEF_FOR_struct_41ZI6UPtvQp2dx0RbZz7gH_

typedef struct {
  real_T MCFL_ActualTorque;
  real_T MCFL_ActualVelocity;
  real_T MCFL_DCVoltage;
  real_T MCFL_bDCOn;
  real_T MCFL_bDerating;
  real_T MCFL_bError;
  real_T MCFL_bInverterOn;
  real_T MCFL_bQuitDCOn;
  real_T MCFL_bQuitInverterOn;
  real_T MCFL_bReserve;
  real_T MCFL_bSystemReady;
  real_T MCFL_bWarn;
} struct_41ZI6UPtvQp2dx0RbZz7gH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_uHl9C1v5uvUP1QsYqLjb5D_
#define DEFINED_TYPEDEF_FOR_struct_uHl9C1v5uvUP1QsYqLjb5D_

typedef struct {
  uint16_T Acc1;
  uint16_T Brk1;
  uint16_T Acc2;
  uint16_T Brk2;
} struct_uHl9C1v5uvUP1QsYqLjb5D;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_K5cKe8WzTvKtCDgoaM96DE_
#define DEFINED_TYPEDEF_FOR_struct_K5cKe8WzTvKtCDgoaM96DE_

typedef struct {
  real_T IMU_Ax_Value;
  real_T IMU_Ay_Value;
  real_T IMU_Yaw_Value;
  real_T IMU_Ay_Valid;
  real_T IMU_DTC;
  real_T IMU_Yaw_Valid;
  real_T IMU_Ax_Valid;
} struct_K5cKe8WzTvKtCDgoaM96DE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_EZ1YTIdnW9R1PG0j9eMP1C_
#define DEFINED_TYPEDEF_FOR_struct_EZ1YTIdnW9R1PG0j9eMP1C_

typedef struct {
  real32_T Acc_POS;
  real_T Trq_CUT;
  real32_T Brk_F;
} struct_EZ1YTIdnW9R1PG0j9eMP1C;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_vEipMMqPBw35USs2kxooHF_
#define DEFINED_TYPEDEF_FOR_struct_vEipMMqPBw35USs2kxooHF_

typedef struct {
  boolean_T WhlStrAng_Valid;
  real32_T FLWhlStrAng;
  real32_T FRWhlStrAng;
  real32_T StrWhlAng;
  real32_T StrWhlAngV;
} struct_vEipMMqPBw35USs2kxooHF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_XaCvvqTqdLQ0rV6twwRd8_
#define DEFINED_TYPEDEF_FOR_struct_XaCvvqTqdLQ0rV6twwRd8_

typedef struct {
  real_T WhlSpdRL_Valid;
  real_T WhlSpdRL_mps;
  real_T WhlSpdRR_Valid;
  real_T WhlSpdRR_mps;
  real_T WhlSpdFR_Valid;
  real_T WhlSpdFR;
  real_T WhlSpdFL_Valid;
  real_T WhlSpdFL;
} struct_XaCvvqTqdLQ0rV6twwRd8;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_PNhhcumawvQ2EWFN2sxbEH_
#define DEFINED_TYPEDEF_FOR_struct_PNhhcumawvQ2EWFN2sxbEH_

typedef struct {
  real_T MCFL_DiagnosticNum;
  real_T MCFL_ErrorInfo;
} struct_PNhhcumawvQ2EWFN2sxbEH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_gpIcNjTnGJzsLFljLm8lnF_
#define DEFINED_TYPEDEF_FOR_struct_gpIcNjTnGJzsLFljLm8lnF_

typedef struct {
  real_T MCFL_TempIGBT;
  real_T MCFL_TempInverter;
  real_T MCFL_TempMotor;
} struct_gpIcNjTnGJzsLFljLm8lnF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_jdp2LVhriOnMZNDiIazdgH_
#define DEFINED_TYPEDEF_FOR_struct_jdp2LVhriOnMZNDiIazdgH_

typedef struct {
  real_T MCFR_ActualTorque;
  real_T MCFR_ActualVelocity;
  real_T MCFR_DCVoltage;
  real_T MCFR_bDCOn;
  real_T MCFR_bDerating;
  real_T MCFR_bError;
  real_T MCFR_bInverterOn;
  real_T MCFR_bQuitDCOn;
  real_T MCFR_bQuitInverterOn;
  real_T MCFR_bReserve;
  real_T MCFR_bSystemReady;
  real_T MCFR_bWarn;
} struct_jdp2LVhriOnMZNDiIazdgH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_HpVbJVWHEDPdi8ouD1ADGB_
#define DEFINED_TYPEDEF_FOR_struct_HpVbJVWHEDPdi8ouD1ADGB_

typedef struct {
  real_T MCFR_DiagnosticNum;
  real_T MCFR_ErrorInfo;
} struct_HpVbJVWHEDPdi8ouD1ADGB;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_6LjBvhA3CbD6qsBMF5t9jG_
#define DEFINED_TYPEDEF_FOR_struct_6LjBvhA3CbD6qsBMF5t9jG_

typedef struct {
  real_T MCFR_TempIGBT;
  real_T MCFR_TempInverter;
  real_T MCFR_TempMotor;
} struct_6LjBvhA3CbD6qsBMF5t9jG;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_1cYjsTlJGVNfp1U53ED5wH_
#define DEFINED_TYPEDEF_FOR_struct_1cYjsTlJGVNfp1U53ED5wH_

typedef struct {
  struct_41ZI6UPtvQp2dx0RbZz7gH FLAMKMCUtoVCU1;
  struct_PNhhcumawvQ2EWFN2sxbEH FLAMKMCUtoVCU2;
  struct_gpIcNjTnGJzsLFljLm8lnF FLAMKMCUtoVCU3;
  struct_jdp2LVhriOnMZNDiIazdgH FRAMKMCUtoVCU1;
  struct_HpVbJVWHEDPdi8ouD1ADGB FRAMKMCUtoVCU2;
  struct_6LjBvhA3CbD6qsBMF5t9jG FRAMKMCUtoVCU3;
} struct_1cYjsTlJGVNfp1U53ED5wH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_n6mEYHY3fl5Mf7PSKiYZcB_
#define DEFINED_TYPEDEF_FOR_struct_n6mEYHY3fl5Mf7PSKiYZcB_

typedef struct {
  real_T StrWhlAng;
  real_T StrWhlAngV;
  real_T StrWhlAngAliveRollCnt;
} struct_n6mEYHY3fl5Mf7PSKiYZcB;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_4YQUTqbEsFrktSsAh2zFNF_
#define DEFINED_TYPEDEF_FOR_struct_4YQUTqbEsFrktSsAh2zFNF_

typedef struct {
  real_T BatAlmlv;
  real_T BatLife;
  real_T BatState;
  real_T LSBAz;
  real_T LSBVz;
  real_T MSBAz;
  real_T MSBVz;
  real_T SOCz;
  real_T SOHz;
} struct_4YQUTqbEsFrktSsAh2zFNF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_oSrQ2LBjXc27gABuWl7wTC_
#define DEFINED_TYPEDEF_FOR_struct_oSrQ2LBjXc27gABuWl7wTC_

typedef struct {
  real_T ABS_WS_FR;
  real_T ABS_WS_RL;
  real_T ABS_WS_FL;
  real_T ABS_WS_RR;
} struct_oSrQ2LBjXc27gABuWl7wTC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_21V72ZoKHO4rPQjhYZWkf_
#define DEFINED_TYPEDEF_FOR_struct_21V72ZoKHO4rPQjhYZWkf_

typedef struct {
  real_T L12V_error;
  real_T alarm;
  real_T low_VOL;
  real_T controller_ready;
  real_T MCU_Temp_error;
  real_T Mode;
  real_T motorTemp_error;
  real_T overCurrent;
  real_T overpower;
  real_T overvol;
  real_T Precharge;
  real_T Reslove_error;
  real_T selfcheck;
  real_T RPM;
  real_T Trq;
} struct_21V72ZoKHO4rPQjhYZWkf;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_JkutEZMu1ZZaoAN93oCNSH_
#define DEFINED_TYPEDEF_FOR_struct_JkutEZMu1ZZaoAN93oCNSH_

typedef struct {
  real_T AC_current;
  real_T MCU_temp;
  real_T current;
  real_T motor_temp;
  real_T voltage;
} struct_JkutEZMu1ZZaoAN93oCNSH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_PrCRZsfUQUgTMs8pMJ2LJD_
#define DEFINED_TYPEDEF_FOR_struct_PrCRZsfUQUgTMs8pMJ2LJD_

typedef struct {
  struct_21V72ZoKHO4rPQjhYZWkf MCU_state;
  struct_JkutEZMu1ZZaoAN93oCNSH MCU_pwr;
} struct_PrCRZsfUQUgTMs8pMJ2LJD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_zHe2tMljVrX4QA50e8w5eH_
#define DEFINED_TYPEDEF_FOR_struct_zHe2tMljVrX4QA50e8w5eH_

typedef struct {
  real_T ignition;
  real_T AMKSWITCH;
  boolean_T out2;
} struct_zHe2tMljVrX4QA50e8w5eH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_STtLlsBt7Hv2iw9aFp36PH_
#define DEFINED_TYPEDEF_FOR_struct_STtLlsBt7Hv2iw9aFp36PH_

typedef struct {
  struct_K5cKe8WzTvKtCDgoaM96DE IMU;
  struct_uHl9C1v5uvUP1QsYqLjb5D AccBrk_BUS;
  struct_1cYjsTlJGVNfp1U53ED5wH AMKMCUtoVCU_BUS;
  struct_n6mEYHY3fl5Mf7PSKiYZcB SWAS;
  struct_4YQUTqbEsFrktSsAh2zFNF BMS_BUS;
  struct_oSrQ2LBjXc27gABuWl7wTC ABS_BUS;
  struct_PrCRZsfUQUgTMs8pMJ2LJD EMRAXMCU_Receive;
  struct_zHe2tMljVrX4QA50e8w5eH KEY;
} struct_STtLlsBt7Hv2iw9aFp36PH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_YoC1NaNpSesc0JfhkfKVDC_
#define DEFINED_TYPEDEF_FOR_struct_YoC1NaNpSesc0JfhkfKVDC_

typedef struct {
  struct_K5cKe8WzTvKtCDgoaM96DE IMU_Processing;
  struct_EZ1YTIdnW9R1PG0j9eMP1C AccBrkPedal_Processing;
  struct_vEipMMqPBw35USs2kxooHF SWAS_Processing;
  struct_XaCvvqTqdLQ0rV6twwRd8 WhlSpdii_mps;
  struct_STtLlsBt7Hv2iw9aFp36PH BUS;
} struct_YoC1NaNpSesc0JfhkfKVDC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_wuK7zwM3QidNa443mIz8QD_
#define DEFINED_TYPEDEF_FOR_struct_wuK7zwM3QidNa443mIz8QD_

typedef struct {
  real32_T VxFL_mps;
  real32_T VxFR_mps;
  real32_T VxRL_mps;
  real32_T VxRR_mps;
} struct_wuK7zwM3QidNa443mIz8QD;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_VehCtrlMdel240926_201_T RT_MODEL_VehCtrlMdel240926_20_T;

#endif             /* RTW_HEADER_VehCtrlMdel240926_2018b_amkspdlimit_types_h_ */

/* File trailer for ECUCoder generated file VehCtrlMdel240926_2018b_amkspdlimit_types.h.
 *
 * [EOF]
 */
