/*
 * Code generated for Simulink model RP20231104WLR_231105.
 *
 * FILE    : RP20231104WLR_231105_types.h
 *
 * VERSION : 1.1
 *
 * DATE    : Sat Sep 21 10:42:01 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#ifndef RTW_HEADER_RP20231104WLR_231105_types_h_
#define RTW_HEADER_RP20231104WLR_231105_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_struct_kzBebJZ7Dzj9HE9JUeOj9C_
#define DEFINED_TYPEDEF_FOR_struct_kzBebJZ7Dzj9HE9JUeOj9C_

typedef struct {
  uint16_T Water_pump;
} struct_kzBebJZ7Dzj9HE9JUeOj9C;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_D6O7YrSW8O3uHtK1aPjY1_
#define DEFINED_TYPEDEF_FOR_struct_D6O7YrSW8O3uHtK1aPjY1_

typedef struct {
  real32_T Trq_cmd;
  real_T RPM_cmd;
  boolean_T TCS_flgActv;
  real_T TCS_TrqRequestFinal_Nm;
} struct_D6O7YrSW8O3uHtK1aPjY1;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_jF0tAqRK6uJ2rc6FZqbVC_
#define DEFINED_TYPEDEF_FOR_struct_jF0tAqRK6uJ2rc6FZqbVC_

typedef struct {
  real32_T Acc_POS;
  boolean_T Trq_CUT;
  real32_T Brk_F;
  real32_T Brk_R;
} struct_jF0tAqRK6uJ2rc6FZqbVC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_OT5N2sw4t55i8ctsXeymDE_
#define DEFINED_TYPEDEF_FOR_struct_OT5N2sw4t55i8ctsXeymDE_

typedef struct {
  boolean_T LVMS;
  boolean_T ignition;
} struct_OT5N2sw4t55i8ctsXeymDE;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_RP20231104WLR_231105_T RT_MODEL_RP20231104WLR_231105_T;

#endif                            /* RTW_HEADER_RP20231104WLR_231105_types_h_ */

/* File trailer for ECUCoder generated file RP20231104WLR_231105_types.h.
 *
 * [EOF]
 */
