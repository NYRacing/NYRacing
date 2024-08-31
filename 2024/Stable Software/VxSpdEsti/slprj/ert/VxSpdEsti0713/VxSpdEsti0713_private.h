/*
 * File: VxSpdEsti0713_private.h
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

#ifndef RTW_HEADER_VxSpdEsti0713_private_h_
#define RTW_HEADER_VxSpdEsti0713_private_h_
#include "rtwtypes.h"
#include "VxSpdEsti0713.h"
#include "VxSpdEsti0713_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         (*((rtm)->errorStatus))
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    (*((rtm)->errorStatus) = (val))
#endif

#ifndef rtmGetErrorStatusPointer
#define rtmGetErrorStatusPointer(rtm)  (rtm)->errorStatus
#endif

#ifndef rtmSetErrorStatusPointer
#define rtmSetErrorStatusPointer(rtm, val) ((rtm)->errorStatus = (val))
#endif

/* Invariant block signals (default storage) */
extern const ConstB_VxSpdEsti0713_h_T VxSpdEsti0713_ConstB;

#endif                                 /* RTW_HEADER_VxSpdEsti0713_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
