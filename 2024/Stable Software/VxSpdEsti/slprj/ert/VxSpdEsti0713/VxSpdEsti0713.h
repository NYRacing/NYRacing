/*
 * File: VxSpdEsti0713.h
 *
 * Code generated for Simulink model 'VxSpdEsti0713'.
 *
 * Model version                  : 1.38
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Sat Jul 13 10:45:08 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: AMD->Athlon 64
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_VxSpdEsti0713_h_
#define RTW_HEADER_VxSpdEsti0713_h_
#ifndef VxSpdEsti0713_COMMON_INCLUDES_
#define VxSpdEsti0713_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* VxSpdEsti0713_COMMON_INCLUDES_ */

#include "VxSpdEsti0713_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Block states (default storage) for model 'VxSpdEsti0713' */
typedef struct {
  real_T UnitDelay_DSTATE;             /* '<S12>/Unit Delay' */
  real_T DelayInput2_DSTATE;           /* '<S16>/Delay Input2' */
  real_T UnitDelay_DSTATE_d;           /* '<S5>/Unit Delay' */
  real_T UnitDelay_DSTATE_k;           /* '<S13>/Unit Delay' */
  real_T DelayInput2_DSTATE_c;         /* '<S19>/Delay Input2' */
  real_T UnitDelay1_DSTATE;            /* '<S5>/Unit Delay1' */
  real_T UnitDelay_DSTATE_h;           /* '<S14>/Unit Delay' */
  real_T DelayInput2_DSTATE_m;         /* '<S22>/Delay Input2' */
  real_T UnitDelay2_DSTATE;            /* '<S5>/Unit Delay2' */
  real_T UnitDelay_DSTATE_j;           /* '<S15>/Unit Delay' */
  real_T DelayInput2_DSTATE_e;         /* '<S25>/Delay Input2' */
  real_T UnitDelay3_DSTATE;            /* '<S5>/Unit Delay3' */
  real_T UnitDelay_DSTATE_dh;          /* '<S32>/Unit Delay' */
  real_T DelayInput2_DSTATE_l;         /* '<S36>/Delay Input2' */
  real_T UnitDelay4_DSTATE;            /* '<S6>/Unit Delay4' */
  real_T UnitDelay_DSTATE_kf;          /* '<S6>/Unit Delay' */
  real_T UnitDelay_DSTATE_l;           /* '<S33>/Unit Delay' */
  real_T DelayInput2_DSTATE_lp;        /* '<S39>/Delay Input2' */
  real_T UnitDelay5_DSTATE;            /* '<S6>/Unit Delay5' */
  real_T UnitDelay1_DSTATE_k;          /* '<S6>/Unit Delay1' */
  real_T UnitDelay_DSTATE_p;           /* '<S34>/Unit Delay' */
  real_T DelayInput2_DSTATE_f;         /* '<S42>/Delay Input2' */
  real_T UnitDelay6_DSTATE;            /* '<S6>/Unit Delay6' */
  real_T UnitDelay2_DSTATE_p;          /* '<S6>/Unit Delay2' */
  real_T UnitDelay_DSTATE_m;           /* '<S35>/Unit Delay' */
  real_T DelayInput2_DSTATE_o;         /* '<S45>/Delay Input2' */
  real_T UnitDelay7_DSTATE;            /* '<S6>/Unit Delay7' */
  real_T UnitDelay3_DSTATE_o;          /* '<S6>/Unit Delay3' */
  real_T UnitDelay_DSTATE_hv;          /* '<S4>/Unit Delay' */
  real32_T UnitDelay_DSTATE_e;         /* '<Root>/Unit Delay' */
  real32_T Delay_DSTATE[5];            /* '<S3>/Delay' */
  real32_T UnitDelay_DSTATE_pp;        /* '<S3>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_i;        /* '<Root>/Unit Delay1' */
  real32_T UnitDelay4_DSTATE_d;        /* '<S3>/Unit Delay4' */
  real32_T UnitDelay2_DSTATE_h;        /* '<S3>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_pz;        /* '<S57>/Unit Delay' */
  real32_T DelayInput2_DSTATE_b;       /* '<S61>/Delay Input2' */
  real32_T DelayInput2_DSTATE_c4;      /* '<S58>/Delay Input2' */
  real32_T UnitDelay_DSTATE_o;         /* '<S56>/Unit Delay' */
  boolean_T UnitDelay3_DSTATE_j;       /* '<S3>/Unit Delay3' */
} DW_VxSpdEsti0713_f_T;

/* Invariant block signals for model 'VxSpdEsti0713' */
typedef struct {
  const real_T Gain;                   /* '<S1>/Gain' */
  const real_T Gain1;                  /* '<S1>/Gain1' */
} ConstB_VxSpdEsti0713_h_T;

/* Real-time Model Data Structure */
struct tag_RTM_VxSpdEsti0713_T {
  const char_T **errorStatus;
};

typedef struct {
  DW_VxSpdEsti0713_f_T rtdw;
  RT_MODEL_VxSpdEsti0713_T rtm;
} MdlrefDW_VxSpdEsti0713_T;

/* Model reference registration function */
extern void VxSpdEsti0713_initialize(const char_T **rt_errorStatus,
  RT_MODEL_VxSpdEsti0713_T *const VxSpdEsti0713_M);
extern void VxSpdEsti0713(const real32_T *rtu_IMU_Ax_mpss, const real32_T
  *rtu_IMU_YawRate_dps, const real32_T *rtu_WhlStrAng_deg, const real32_T
  *rtu_TrqReq_Nm, const real32_T *rtu_WhlSpdFL_mps, const real32_T
  *rtu_WhlSpdFR_mps, const real32_T *rtu_WhlSpdRL_mps, const real32_T
  *rtu_WhlSpdRR_mps, real32_T *rty_VehVxEst_mps, DW_VxSpdEsti0713_f_T *localDW);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S16>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S18>/Data Type Duplicate' : Unused code path elimination
 * Block '<S18>/Data Type Propagation' : Unused code path elimination
 * Block '<S17>/Data Type Duplicate' : Unused code path elimination
 * Block '<S17>/Data Type Propagation' : Unused code path elimination
 * Block '<S19>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S21>/Data Type Duplicate' : Unused code path elimination
 * Block '<S21>/Data Type Propagation' : Unused code path elimination
 * Block '<S20>/Data Type Duplicate' : Unused code path elimination
 * Block '<S20>/Data Type Propagation' : Unused code path elimination
 * Block '<S22>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S24>/Data Type Duplicate' : Unused code path elimination
 * Block '<S24>/Data Type Propagation' : Unused code path elimination
 * Block '<S23>/Data Type Duplicate' : Unused code path elimination
 * Block '<S23>/Data Type Propagation' : Unused code path elimination
 * Block '<S25>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S27>/Data Type Duplicate' : Unused code path elimination
 * Block '<S27>/Data Type Propagation' : Unused code path elimination
 * Block '<S26>/Data Type Duplicate' : Unused code path elimination
 * Block '<S26>/Data Type Propagation' : Unused code path elimination
 * Block '<S36>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S38>/Data Type Duplicate' : Unused code path elimination
 * Block '<S38>/Data Type Propagation' : Unused code path elimination
 * Block '<S37>/Data Type Duplicate' : Unused code path elimination
 * Block '<S37>/Data Type Propagation' : Unused code path elimination
 * Block '<S39>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S41>/Data Type Duplicate' : Unused code path elimination
 * Block '<S41>/Data Type Propagation' : Unused code path elimination
 * Block '<S40>/Data Type Duplicate' : Unused code path elimination
 * Block '<S40>/Data Type Propagation' : Unused code path elimination
 * Block '<S42>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S44>/Data Type Duplicate' : Unused code path elimination
 * Block '<S44>/Data Type Propagation' : Unused code path elimination
 * Block '<S43>/Data Type Duplicate' : Unused code path elimination
 * Block '<S43>/Data Type Propagation' : Unused code path elimination
 * Block '<S45>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S47>/Data Type Duplicate' : Unused code path elimination
 * Block '<S47>/Data Type Propagation' : Unused code path elimination
 * Block '<S46>/Data Type Duplicate' : Unused code path elimination
 * Block '<S46>/Data Type Propagation' : Unused code path elimination
 * Block '<S58>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Propagation' : Unused code path elimination
 * Block '<S59>/Data Type Duplicate' : Unused code path elimination
 * Block '<S59>/Data Type Propagation' : Unused code path elimination
 * Block '<S61>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S63>/Data Type Duplicate' : Unused code path elimination
 * Block '<S63>/Data Type Propagation' : Unused code path elimination
 * Block '<S62>/Data Type Duplicate' : Unused code path elimination
 * Block '<S62>/Data Type Propagation' : Unused code path elimination
 * Block '<Root>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<Root>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<Root>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<Root>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<Root>/Cast To Double4' : Eliminate redundant data type conversion
 * Block '<Root>/Cast To Double5' : Eliminate redundant data type conversion
 * Block '<Root>/Cast To Double6' : Eliminate redundant data type conversion
 * Block '<S2>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S2>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S2>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S2>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S3>/Cast To Double' : Eliminate redundant data type conversion
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
 * '<Root>' : 'VxSpdEsti0713'
 * '<S1>'   : 'VxSpdEsti0713/坐标系转换'
 * '<S2>'   : 'VxSpdEsti0713/打滑判断'
 * '<S3>'   : 'VxSpdEsti0713/积分判断'
 * '<S4>'   : 'VxSpdEsti0713/车速估计'
 * '<S5>'   : 'VxSpdEsti0713/打滑判断/加速度差判断'
 * '<S6>'   : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断'
 * '<S7>'   : 'VxSpdEsti0713/打滑判断/速差判断'
 * '<S8>'   : 'VxSpdEsti0713/打滑判断/加速度差判断/Compare To Constant'
 * '<S9>'   : 'VxSpdEsti0713/打滑判断/加速度差判断/Compare To Constant1'
 * '<S10>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Compare To Constant2'
 * '<S11>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Compare To Constant3'
 * '<S12>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter'
 * '<S13>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter1'
 * '<S14>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter2'
 * '<S15>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter3'
 * '<S16>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter/Rate Limiter Dynamic'
 * '<S17>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter/Saturation Dynamic'
 * '<S18>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S19>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter1/Rate Limiter Dynamic'
 * '<S20>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter1/Saturation Dynamic'
 * '<S21>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S22>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter2/Rate Limiter Dynamic'
 * '<S23>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter2/Saturation Dynamic'
 * '<S24>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S25>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter3/Rate Limiter Dynamic'
 * '<S26>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter3/Saturation Dynamic'
 * '<S27>'  : 'VxSpdEsti0713/打滑判断/加速度差判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S28>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Compare To Constant'
 * '<S29>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Compare To Constant1'
 * '<S30>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Compare To Constant2'
 * '<S31>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Compare To Constant3'
 * '<S32>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter'
 * '<S33>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter1'
 * '<S34>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter2'
 * '<S35>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter3'
 * '<S36>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter/Rate Limiter Dynamic'
 * '<S37>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter/Saturation Dynamic'
 * '<S38>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S39>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter1/Rate Limiter Dynamic'
 * '<S40>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter1/Saturation Dynamic'
 * '<S41>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S42>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter2/Rate Limiter Dynamic'
 * '<S43>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter2/Saturation Dynamic'
 * '<S44>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S45>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter3/Rate Limiter Dynamic'
 * '<S46>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter3/Saturation Dynamic'
 * '<S47>'  : 'VxSpdEsti0713/打滑判断/加速度差的微分的判断/Filter3/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S48>'  : 'VxSpdEsti0713/打滑判断/速差判断/Compare To Constant'
 * '<S49>'  : 'VxSpdEsti0713/打滑判断/速差判断/Compare To Constant1'
 * '<S50>'  : 'VxSpdEsti0713/打滑判断/速差判断/Compare To Constant2'
 * '<S51>'  : 'VxSpdEsti0713/打滑判断/速差判断/Compare To Constant3'
 * '<S52>'  : 'VxSpdEsti0713/积分判断/Compare To Constant'
 * '<S53>'  : 'VxSpdEsti0713/积分判断/Compare To Constant1'
 * '<S54>'  : 'VxSpdEsti0713/车速估计/Compare To Constant'
 * '<S55>'  : 'VxSpdEsti0713/车速估计/Compare To Constant1'
 * '<S56>'  : 'VxSpdEsti0713/车速估计/Filter'
 * '<S57>'  : 'VxSpdEsti0713/车速估计/Filter1'
 * '<S58>'  : 'VxSpdEsti0713/车速估计/Filter/Rate Limiter Dynamic'
 * '<S59>'  : 'VxSpdEsti0713/车速估计/Filter/Saturation Dynamic'
 * '<S60>'  : 'VxSpdEsti0713/车速估计/Filter/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S61>'  : 'VxSpdEsti0713/车速估计/Filter1/Rate Limiter Dynamic'
 * '<S62>'  : 'VxSpdEsti0713/车速估计/Filter1/Saturation Dynamic'
 * '<S63>'  : 'VxSpdEsti0713/车速估计/Filter1/Rate Limiter Dynamic/Saturation Dynamic'
 */
#endif                                 /* RTW_HEADER_VxSpdEsti0713_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
