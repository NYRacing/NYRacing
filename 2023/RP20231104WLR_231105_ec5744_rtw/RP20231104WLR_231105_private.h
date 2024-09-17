/*
 * Code generated for Simulink model RP20231104WLR_231105.
 *
 * FILE    : RP20231104WLR_231105_private.h
 *
 * VERSION : 1.1
 *
 * DATE    : Wed Sep 18 01:37:22 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#ifndef RTW_HEADER_RP20231104WLR_231105_private_h_
#define RTW_HEADER_RP20231104WLR_231105_private_h_
#include "rtwtypes.h"
#include "RP20231104WLR_231105.h"
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

/* #define mydefine 100.0 */
extern uint32_T look1_iu32n16bflftfIu32_binlc(uint32_T u0, const real32_T bp0[],
  const real32_T table[], uint32_T maxIndex);
extern uint32_T look1_iu32bflftfIu32_binlc(uint32_T u0, const real32_T bp0[],
  const real32_T table[], uint32_T maxIndex);
extern real32_T look2_iflf_binlx(real32_T u0, real32_T u1, const real32_T bp0[],
  const real32_T bp1[], const real32_T table[], const uint32_T maxIndex[],
  uint32_T stride);
extern real_T look1_binlc(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex);
extern real32_T look1_iflf_binlc(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex);
extern void RP20231104WLR_231105_step0(void);
extern void RP20231104WLR_231105_step1(void);
extern void RP20231104WLR_231105_step2(void);
extern void RP20231104WLR_231105_step3(void);
extern void RP20231104WLR_231105_step4(void);
extern void RP20231104WLR_231105_step5(void);
extern void RP20231104WLR_231105_step6(void);

#endif                          /* RTW_HEADER_RP20231104WLR_231105_private_h_ */

/* File trailer for ECUCoder generated file RP20231104WLR_231105_private.h.
 *
 * [EOF]
 */
