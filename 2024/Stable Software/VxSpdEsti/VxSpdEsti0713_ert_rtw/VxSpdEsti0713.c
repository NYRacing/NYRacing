/*
 * File: VxSpdEsti0713.c
 *
 * Code generated for Simulink model 'VxSpdEsti0713'.
 *
 * Model version                  : 1.37
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Sat Jul 13 10:27:14 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "VxSpdEsti0713.h"
#include <math.h>
#include "rtwtypes.h"
#include "rt_nonfinite.h"

/* Block states (default storage) */
DW_VxSpdEsti0713_T VxSpdEsti0713_DW;

/* External inputs (root inport signals with default storage) */
ExtU_VxSpdEsti0713_T VxSpdEsti0713_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_VxSpdEsti0713_T VxSpdEsti0713_Y;

/* Real-time model */
static RT_MODEL_VxSpdEsti0713_T VxSpdEsti0713_M_;
RT_MODEL_VxSpdEsti0713_T *const VxSpdEsti0713_M = &VxSpdEsti0713_M_;

/* Model step function */
void VxSpdEsti0713_step(void)
{
  real_T rtb_Add14;
  real_T rtb_Divide;
  real_T rtb_Divide2;
  real_T rtb_Divide3;
  real_T rtb_Divide3_b;
  real_T rtb_Gain;
  real_T rtb_Gain_g;
  real_T rtb_Gain_gu;
  real_T rtb_Min1_j;
  real_T rtb_UnitDelay4;
  real_T rtb_UnitDelay5;
  real_T rtb_UnitDelay6;
  real_T rtb_UnitDelay7;
  real_T rtb_VxFL_mps;
  real32_T rtb_Add1_f;
  real32_T rtb_Switch2_k;
  real32_T rtb_UkYk1_nb;
  boolean_T rtb_LogicalOperator1_l_idx_0;
  boolean_T rtb_LogicalOperator1_l_idx_1;
  boolean_T rtb_LogicalOperator1_l_idx_2;
  boolean_T rtb_LogicalOperator1_l_idx_3;
  boolean_T rtb_LogicalOperator2;

  /* Gain: '<S12>/Gain' incorporates:
   *  UnitDelay: '<S12>/Unit Delay'
   */
  rtb_Gain = 0.3 * VxSpdEsti0713_DW.UnitDelay_DSTATE;

  /* Product: '<S16>/delta rise limit' incorporates:
   *  Constant: '<S12>/Constant2'
   *  SampleTimeMath: '<S16>/sample time'
   *
   * About '<S16>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Divide3_b = 1.0;

  /* Trigonometry: '<S1>/Cos' incorporates:
   *  Inport: '<Root>/WhlStrAng_deg'
   */
  rtb_Add1_f = cosf(VxSpdEsti0713_U.WhlStrAng_deg);

  /* Product: '<S1>/Product3' incorporates:
   *  Inport: '<Root>/IMU_YawRate_dps'
   */
  rtb_Add14 = VxSpdEsti0713_ConstB.Gain1 * VxSpdEsti0713_U.IMU_YawRate_dps;

  /* Sum: '<S1>/Add2' incorporates:
   *  Inport: '<Root>/In1'
   *  Product: '<S1>/Product1'
   */
  rtb_VxFL_mps = VxSpdEsti0713_U.In1 * rtb_Add1_f - rtb_Add14;

  /* Product: '<S5>/Divide' incorporates:
   *  Sum: '<S5>/Add4'
   *  UnitDelay: '<S5>/Unit Delay'
   */
  rtb_Divide = (rtb_VxFL_mps - VxSpdEsti0713_DW.UnitDelay_DSTATE_d) /
    0.0099999997764825821;

  /* Switch: '<S17>/Switch2' incorporates:
   *  Constant: '<S12>/Constant1'
   *  RelationalOperator: '<S17>/LowerRelop1'
   *  RelationalOperator: '<S17>/UpperRelop'
   *  Switch: '<S17>/Switch'
   */
  if (rtb_Divide > 100.0) {
    rtb_Divide = 100.0;
  } else if (rtb_Divide < -100.0) {
    /* Switch: '<S17>/Switch' incorporates:
     *  Constant: '<S12>/Constant'
     */
    rtb_Divide = -100.0;
  }

  /* Sum: '<S16>/Difference Inputs1' incorporates:
   *  Switch: '<S17>/Switch2'
   *  UnitDelay: '<S16>/Delay Input2'
   *
   * Block description for '<S16>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S16>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Divide -= VxSpdEsti0713_DW.DelayInput2_DSTATE;

  /* Switch: '<S18>/Switch2' incorporates:
   *  RelationalOperator: '<S18>/LowerRelop1'
   */
  if (!(rtb_Divide > 1.0)) {
    /* Switch: '<S18>/Switch' incorporates:
     *  RelationalOperator: '<S18>/UpperRelop'
     */
    if (rtb_Divide < -1.0) {
      rtb_Divide3_b = -1.0;
    } else {
      rtb_Divide3_b = rtb_Divide;
    }

    /* End of Switch: '<S18>/Switch' */
  }

  /* End of Switch: '<S18>/Switch2' */

  /* Sum: '<S16>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S16>/Delay Input2'
   *
   * Block description for '<S16>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S16>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VxSpdEsti0713_DW.DelayInput2_DSTATE += rtb_Divide3_b;

  /* Saturate: '<S12>/Saturation' incorporates:
   *  UnitDelay: '<S16>/Delay Input2'
   *
   * Block description for '<S16>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (VxSpdEsti0713_DW.DelayInput2_DSTATE > 0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE = 0.5;
  } else if (VxSpdEsti0713_DW.DelayInput2_DSTATE < -0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE = -0.5;
  } else {
    VxSpdEsti0713_DW.UnitDelay_DSTATE = VxSpdEsti0713_DW.DelayInput2_DSTATE;
  }

  /* End of Saturate: '<S12>/Saturation' */

  /* Gain: '<S13>/Gain' incorporates:
   *  UnitDelay: '<S13>/Unit Delay'
   */
  rtb_Gain_g = 0.3 * VxSpdEsti0713_DW.UnitDelay_DSTATE_k;

  /* Product: '<S19>/delta rise limit' incorporates:
   *  Constant: '<S13>/Constant2'
   *  SampleTimeMath: '<S19>/sample time'
   *
   * About '<S19>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Min1_j = 1.0;

  /* Sum: '<S1>/Add3' incorporates:
   *  Inport: '<Root>/In2'
   *  Product: '<S1>/Product2'
   */
  rtb_Divide3_b = rtb_Add1_f * VxSpdEsti0713_U.In2 + rtb_Add14;

  /* Product: '<S5>/Divide1' incorporates:
   *  Sum: '<S5>/Add5'
   *  UnitDelay: '<S5>/Unit Delay1'
   */
  rtb_Divide = (rtb_Divide3_b - VxSpdEsti0713_DW.UnitDelay1_DSTATE) /
    0.0099999997764825821;

  /* Switch: '<S20>/Switch2' incorporates:
   *  Constant: '<S13>/Constant1'
   *  RelationalOperator: '<S20>/LowerRelop1'
   *  RelationalOperator: '<S20>/UpperRelop'
   *  Switch: '<S20>/Switch'
   */
  if (rtb_Divide > 100.0) {
    rtb_Divide = 100.0;
  } else if (rtb_Divide < -100.0) {
    /* Switch: '<S20>/Switch' incorporates:
     *  Constant: '<S13>/Constant'
     */
    rtb_Divide = -100.0;
  }

  /* Sum: '<S19>/Difference Inputs1' incorporates:
   *  Switch: '<S20>/Switch2'
   *  UnitDelay: '<S19>/Delay Input2'
   *
   * Block description for '<S19>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S19>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Divide -= VxSpdEsti0713_DW.DelayInput2_DSTATE_c;

  /* Switch: '<S21>/Switch2' incorporates:
   *  RelationalOperator: '<S21>/LowerRelop1'
   */
  if (!(rtb_Divide > 1.0)) {
    /* Switch: '<S21>/Switch' incorporates:
     *  RelationalOperator: '<S21>/UpperRelop'
     */
    if (rtb_Divide < -1.0) {
      rtb_Min1_j = -1.0;
    } else {
      rtb_Min1_j = rtb_Divide;
    }

    /* End of Switch: '<S21>/Switch' */
  }

  /* End of Switch: '<S21>/Switch2' */

  /* Sum: '<S19>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S19>/Delay Input2'
   *
   * Block description for '<S19>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S19>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VxSpdEsti0713_DW.DelayInput2_DSTATE_c += rtb_Min1_j;

  /* Saturate: '<S13>/Saturation' incorporates:
   *  UnitDelay: '<S19>/Delay Input2'
   *
   * Block description for '<S19>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (VxSpdEsti0713_DW.DelayInput2_DSTATE_c > 0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_k = 0.5;
  } else if (VxSpdEsti0713_DW.DelayInput2_DSTATE_c < -0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_k = -0.5;
  } else {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_k = VxSpdEsti0713_DW.DelayInput2_DSTATE_c;
  }

  /* End of Saturate: '<S13>/Saturation' */

  /* Gain: '<S14>/Gain' incorporates:
   *  UnitDelay: '<S14>/Unit Delay'
   */
  rtb_Gain_gu = 0.3 * VxSpdEsti0713_DW.UnitDelay_DSTATE_h;

  /* Product: '<S22>/delta rise limit' incorporates:
   *  Constant: '<S14>/Constant2'
   *  SampleTimeMath: '<S22>/sample time'
   *
   * About '<S22>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Min1_j = 1.0;

  /* Product: '<S1>/Product' incorporates:
   *  Inport: '<Root>/IMU_YawRate_dps'
   */
  rtb_Add14 = VxSpdEsti0713_U.IMU_YawRate_dps * VxSpdEsti0713_ConstB.Gain;

  /* Sum: '<S1>/Add' incorporates:
   *  Inport: '<Root>/In3'
   */
  rtb_Divide = VxSpdEsti0713_U.In3 - rtb_Add14;

  /* Product: '<S5>/Divide2' incorporates:
   *  Sum: '<S5>/Add6'
   *  UnitDelay: '<S5>/Unit Delay2'
   */
  rtb_Divide2 = (rtb_Divide - VxSpdEsti0713_DW.UnitDelay2_DSTATE) /
    0.0099999997764825821;

  /* Switch: '<S23>/Switch2' incorporates:
   *  Constant: '<S14>/Constant1'
   *  RelationalOperator: '<S23>/LowerRelop1'
   *  RelationalOperator: '<S23>/UpperRelop'
   *  Switch: '<S23>/Switch'
   */
  if (rtb_Divide2 > 100.0) {
    rtb_Divide2 = 100.0;
  } else if (rtb_Divide2 < -100.0) {
    /* Switch: '<S23>/Switch' incorporates:
     *  Constant: '<S14>/Constant'
     */
    rtb_Divide2 = -100.0;
  }

  /* Sum: '<S22>/Difference Inputs1' incorporates:
   *  Switch: '<S23>/Switch2'
   *  UnitDelay: '<S22>/Delay Input2'
   *
   * Block description for '<S22>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S22>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Divide2 -= VxSpdEsti0713_DW.DelayInput2_DSTATE_m;

  /* Switch: '<S24>/Switch2' incorporates:
   *  RelationalOperator: '<S24>/LowerRelop1'
   */
  if (!(rtb_Divide2 > 1.0)) {
    /* Switch: '<S24>/Switch' incorporates:
     *  RelationalOperator: '<S24>/UpperRelop'
     */
    if (rtb_Divide2 < -1.0) {
      rtb_Min1_j = -1.0;
    } else {
      rtb_Min1_j = rtb_Divide2;
    }

    /* End of Switch: '<S24>/Switch' */
  }

  /* End of Switch: '<S24>/Switch2' */

  /* Sum: '<S22>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S22>/Delay Input2'
   *
   * Block description for '<S22>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S22>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VxSpdEsti0713_DW.DelayInput2_DSTATE_m += rtb_Min1_j;

  /* Saturate: '<S14>/Saturation' incorporates:
   *  UnitDelay: '<S22>/Delay Input2'
   *
   * Block description for '<S22>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (VxSpdEsti0713_DW.DelayInput2_DSTATE_m > 0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_h = 0.5;
  } else if (VxSpdEsti0713_DW.DelayInput2_DSTATE_m < -0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_h = -0.5;
  } else {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_h = VxSpdEsti0713_DW.DelayInput2_DSTATE_m;
  }

  /* End of Saturate: '<S14>/Saturation' */

  /* Gain: '<S15>/Gain' incorporates:
   *  UnitDelay: '<S15>/Unit Delay'
   */
  rtb_Divide2 = 0.3 * VxSpdEsti0713_DW.UnitDelay_DSTATE_j;

  /* Product: '<S25>/delta rise limit' incorporates:
   *  Constant: '<S15>/Constant2'
   *  SampleTimeMath: '<S25>/sample time'
   *
   * About '<S25>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Min1_j = 1.0;

  /* Sum: '<S1>/Add1' incorporates:
   *  Inport: '<Root>/In4'
   */
  rtb_Add14 += VxSpdEsti0713_U.In4;

  /* Product: '<S5>/Divide3' incorporates:
   *  Sum: '<S5>/Add7'
   *  UnitDelay: '<S5>/Unit Delay3'
   */
  rtb_Divide3 = (rtb_Add14 - VxSpdEsti0713_DW.UnitDelay3_DSTATE) /
    0.0099999997764825821;

  /* Switch: '<S26>/Switch2' incorporates:
   *  Constant: '<S15>/Constant1'
   *  RelationalOperator: '<S26>/LowerRelop1'
   *  RelationalOperator: '<S26>/UpperRelop'
   *  Switch: '<S26>/Switch'
   */
  if (rtb_Divide3 > 100.0) {
    rtb_Divide3 = 100.0;
  } else if (rtb_Divide3 < -100.0) {
    /* Switch: '<S26>/Switch' incorporates:
     *  Constant: '<S15>/Constant'
     */
    rtb_Divide3 = -100.0;
  }

  /* Sum: '<S25>/Difference Inputs1' incorporates:
   *  Switch: '<S26>/Switch2'
   *  UnitDelay: '<S25>/Delay Input2'
   *
   * Block description for '<S25>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S25>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Divide3 -= VxSpdEsti0713_DW.DelayInput2_DSTATE_e;

  /* Switch: '<S27>/Switch2' incorporates:
   *  RelationalOperator: '<S27>/LowerRelop1'
   */
  if (!(rtb_Divide3 > 1.0)) {
    /* Switch: '<S27>/Switch' incorporates:
     *  RelationalOperator: '<S27>/UpperRelop'
     */
    if (rtb_Divide3 < -1.0) {
      rtb_Min1_j = -1.0;
    } else {
      rtb_Min1_j = rtb_Divide3;
    }

    /* End of Switch: '<S27>/Switch' */
  }

  /* End of Switch: '<S27>/Switch2' */

  /* Sum: '<S25>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S25>/Delay Input2'
   *
   * Block description for '<S25>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S25>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VxSpdEsti0713_DW.DelayInput2_DSTATE_e += rtb_Min1_j;

  /* Saturate: '<S15>/Saturation' incorporates:
   *  UnitDelay: '<S25>/Delay Input2'
   *
   * Block description for '<S25>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (VxSpdEsti0713_DW.DelayInput2_DSTATE_e > 0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_j = 0.5;
  } else if (VxSpdEsti0713_DW.DelayInput2_DSTATE_e < -0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_j = -0.5;
  } else {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_j = VxSpdEsti0713_DW.DelayInput2_DSTATE_e;
  }

  /* End of Saturate: '<S15>/Saturation' */

  /* Gain: '<S32>/Gain' incorporates:
   *  UnitDelay: '<S32>/Unit Delay'
   */
  rtb_Divide3 = 0.3 * VxSpdEsti0713_DW.UnitDelay_DSTATE_dh;

  /* Product: '<S36>/delta rise limit' incorporates:
   *  Constant: '<S32>/Constant2'
   *  SampleTimeMath: '<S36>/sample time'
   *
   * About '<S36>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Min1_j = 1.0;

  /* UnitDelay: '<S6>/Unit Delay4' */
  rtb_UnitDelay4 = VxSpdEsti0713_DW.UnitDelay4_DSTATE;

  /* Sum: '<S6>/Add' incorporates:
   *  Inport: '<Root>/IMU_Ax_mpss'
   *  Product: '<S6>/Divide'
   *  Sum: '<S6>/Add4'
   *  UnitDelay: '<S6>/Unit Delay'
   *  UnitDelay: '<S6>/Unit Delay4'
   */
  VxSpdEsti0713_DW.UnitDelay4_DSTATE = (rtb_VxFL_mps -
    VxSpdEsti0713_DW.UnitDelay_DSTATE_kf) / 0.0099999997764825821 -
    VxSpdEsti0713_U.IMU_Ax_mpss;

  /* Product: '<S6>/Divide4' incorporates:
   *  Sum: '<S6>/Add8'
   *  UnitDelay: '<S6>/Unit Delay4'
   */
  rtb_UnitDelay4 = (VxSpdEsti0713_DW.UnitDelay4_DSTATE - rtb_UnitDelay4) /
    0.0099999997764825821;

  /* Switch: '<S37>/Switch2' incorporates:
   *  Constant: '<S32>/Constant1'
   *  RelationalOperator: '<S37>/LowerRelop1'
   *  RelationalOperator: '<S37>/UpperRelop'
   *  Switch: '<S37>/Switch'
   */
  if (rtb_UnitDelay4 > 100.0) {
    rtb_UnitDelay4 = 100.0;
  } else if (rtb_UnitDelay4 < -100.0) {
    /* Switch: '<S37>/Switch' incorporates:
     *  Constant: '<S32>/Constant'
     */
    rtb_UnitDelay4 = -100.0;
  }

  /* Sum: '<S36>/Difference Inputs1' incorporates:
   *  Switch: '<S37>/Switch2'
   *  UnitDelay: '<S36>/Delay Input2'
   *
   * Block description for '<S36>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S36>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UnitDelay4 -= VxSpdEsti0713_DW.DelayInput2_DSTATE_l;

  /* Switch: '<S38>/Switch2' incorporates:
   *  RelationalOperator: '<S38>/LowerRelop1'
   */
  if (!(rtb_UnitDelay4 > 1.0)) {
    /* Switch: '<S38>/Switch' incorporates:
     *  RelationalOperator: '<S38>/UpperRelop'
     */
    if (rtb_UnitDelay4 < -1.0) {
      rtb_Min1_j = -1.0;
    } else {
      rtb_Min1_j = rtb_UnitDelay4;
    }

    /* End of Switch: '<S38>/Switch' */
  }

  /* End of Switch: '<S38>/Switch2' */

  /* Sum: '<S36>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S36>/Delay Input2'
   *
   * Block description for '<S36>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S36>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VxSpdEsti0713_DW.DelayInput2_DSTATE_l += rtb_Min1_j;

  /* Saturate: '<S32>/Saturation' incorporates:
   *  UnitDelay: '<S36>/Delay Input2'
   *
   * Block description for '<S36>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (VxSpdEsti0713_DW.DelayInput2_DSTATE_l > 0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_dh = 0.5;
  } else if (VxSpdEsti0713_DW.DelayInput2_DSTATE_l < -0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_dh = -0.5;
  } else {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_dh = VxSpdEsti0713_DW.DelayInput2_DSTATE_l;
  }

  /* End of Saturate: '<S32>/Saturation' */

  /* Gain: '<S33>/Gain' incorporates:
   *  UnitDelay: '<S33>/Unit Delay'
   */
  rtb_UnitDelay4 = 0.3 * VxSpdEsti0713_DW.UnitDelay_DSTATE_l;

  /* Product: '<S39>/delta rise limit' incorporates:
   *  Constant: '<S33>/Constant2'
   *  SampleTimeMath: '<S39>/sample time'
   *
   * About '<S39>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Min1_j = 1.0;

  /* UnitDelay: '<S6>/Unit Delay5' */
  rtb_UnitDelay5 = VxSpdEsti0713_DW.UnitDelay5_DSTATE;

  /* Sum: '<S6>/Add1' incorporates:
   *  Inport: '<Root>/IMU_Ax_mpss'
   *  Product: '<S6>/Divide1'
   *  Sum: '<S6>/Add5'
   *  UnitDelay: '<S6>/Unit Delay1'
   *  UnitDelay: '<S6>/Unit Delay5'
   */
  VxSpdEsti0713_DW.UnitDelay5_DSTATE = (rtb_Divide3_b -
    VxSpdEsti0713_DW.UnitDelay1_DSTATE_k) / 0.0099999997764825821 -
    VxSpdEsti0713_U.IMU_Ax_mpss;

  /* Product: '<S6>/Divide5' incorporates:
   *  Sum: '<S6>/Add10'
   *  UnitDelay: '<S6>/Unit Delay5'
   */
  rtb_UnitDelay5 = (VxSpdEsti0713_DW.UnitDelay5_DSTATE - rtb_UnitDelay5) /
    0.0099999997764825821;

  /* Switch: '<S40>/Switch2' incorporates:
   *  Constant: '<S33>/Constant1'
   *  RelationalOperator: '<S40>/LowerRelop1'
   *  RelationalOperator: '<S40>/UpperRelop'
   *  Switch: '<S40>/Switch'
   */
  if (rtb_UnitDelay5 > 100.0) {
    rtb_UnitDelay5 = 100.0;
  } else if (rtb_UnitDelay5 < -100.0) {
    /* Switch: '<S40>/Switch' incorporates:
     *  Constant: '<S33>/Constant'
     */
    rtb_UnitDelay5 = -100.0;
  }

  /* Sum: '<S39>/Difference Inputs1' incorporates:
   *  Switch: '<S40>/Switch2'
   *  UnitDelay: '<S39>/Delay Input2'
   *
   * Block description for '<S39>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S39>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UnitDelay5 -= VxSpdEsti0713_DW.DelayInput2_DSTATE_lp;

  /* Switch: '<S41>/Switch2' incorporates:
   *  RelationalOperator: '<S41>/LowerRelop1'
   */
  if (!(rtb_UnitDelay5 > 1.0)) {
    /* Switch: '<S41>/Switch' incorporates:
     *  RelationalOperator: '<S41>/UpperRelop'
     */
    if (rtb_UnitDelay5 < -1.0) {
      rtb_Min1_j = -1.0;
    } else {
      rtb_Min1_j = rtb_UnitDelay5;
    }

    /* End of Switch: '<S41>/Switch' */
  }

  /* End of Switch: '<S41>/Switch2' */

  /* Sum: '<S39>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S39>/Delay Input2'
   *
   * Block description for '<S39>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S39>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VxSpdEsti0713_DW.DelayInput2_DSTATE_lp += rtb_Min1_j;

  /* Saturate: '<S33>/Saturation' incorporates:
   *  UnitDelay: '<S39>/Delay Input2'
   *
   * Block description for '<S39>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (VxSpdEsti0713_DW.DelayInput2_DSTATE_lp > 0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_l = 0.5;
  } else if (VxSpdEsti0713_DW.DelayInput2_DSTATE_lp < -0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_l = -0.5;
  } else {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_l = VxSpdEsti0713_DW.DelayInput2_DSTATE_lp;
  }

  /* End of Saturate: '<S33>/Saturation' */

  /* Gain: '<S34>/Gain' incorporates:
   *  UnitDelay: '<S34>/Unit Delay'
   */
  rtb_UnitDelay5 = 0.3 * VxSpdEsti0713_DW.UnitDelay_DSTATE_p;

  /* Product: '<S42>/delta rise limit' incorporates:
   *  Constant: '<S34>/Constant2'
   *  SampleTimeMath: '<S42>/sample time'
   *
   * About '<S42>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Min1_j = 1.0;

  /* UnitDelay: '<S6>/Unit Delay6' */
  rtb_UnitDelay6 = VxSpdEsti0713_DW.UnitDelay6_DSTATE;

  /* Sum: '<S6>/Add2' incorporates:
   *  Inport: '<Root>/IMU_Ax_mpss'
   *  Product: '<S6>/Divide2'
   *  Sum: '<S6>/Add6'
   *  UnitDelay: '<S6>/Unit Delay2'
   *  UnitDelay: '<S6>/Unit Delay6'
   */
  VxSpdEsti0713_DW.UnitDelay6_DSTATE = (rtb_Divide -
    VxSpdEsti0713_DW.UnitDelay2_DSTATE_p) / 0.0099999997764825821 -
    VxSpdEsti0713_U.IMU_Ax_mpss;

  /* Product: '<S6>/Divide6' incorporates:
   *  Sum: '<S6>/Add12'
   *  UnitDelay: '<S6>/Unit Delay6'
   */
  rtb_UnitDelay6 = (VxSpdEsti0713_DW.UnitDelay6_DSTATE - rtb_UnitDelay6) /
    0.0099999997764825821;

  /* Switch: '<S43>/Switch2' incorporates:
   *  Constant: '<S34>/Constant1'
   *  RelationalOperator: '<S43>/LowerRelop1'
   *  RelationalOperator: '<S43>/UpperRelop'
   *  Switch: '<S43>/Switch'
   */
  if (rtb_UnitDelay6 > 100.0) {
    rtb_UnitDelay6 = 100.0;
  } else if (rtb_UnitDelay6 < -100.0) {
    /* Switch: '<S43>/Switch' incorporates:
     *  Constant: '<S34>/Constant'
     */
    rtb_UnitDelay6 = -100.0;
  }

  /* Sum: '<S42>/Difference Inputs1' incorporates:
   *  Switch: '<S43>/Switch2'
   *  UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UnitDelay6 -= VxSpdEsti0713_DW.DelayInput2_DSTATE_f;

  /* Switch: '<S44>/Switch2' incorporates:
   *  RelationalOperator: '<S44>/LowerRelop1'
   */
  if (!(rtb_UnitDelay6 > 1.0)) {
    /* Switch: '<S44>/Switch' incorporates:
     *  RelationalOperator: '<S44>/UpperRelop'
     */
    if (rtb_UnitDelay6 < -1.0) {
      rtb_Min1_j = -1.0;
    } else {
      rtb_Min1_j = rtb_UnitDelay6;
    }

    /* End of Switch: '<S44>/Switch' */
  }

  /* End of Switch: '<S44>/Switch2' */

  /* Sum: '<S42>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VxSpdEsti0713_DW.DelayInput2_DSTATE_f += rtb_Min1_j;

  /* Saturate: '<S34>/Saturation' incorporates:
   *  UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (VxSpdEsti0713_DW.DelayInput2_DSTATE_f > 0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_p = 0.5;
  } else if (VxSpdEsti0713_DW.DelayInput2_DSTATE_f < -0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_p = -0.5;
  } else {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_p = VxSpdEsti0713_DW.DelayInput2_DSTATE_f;
  }

  /* End of Saturate: '<S34>/Saturation' */

  /* Gain: '<S35>/Gain' incorporates:
   *  UnitDelay: '<S35>/Unit Delay'
   */
  rtb_UnitDelay6 = 0.3 * VxSpdEsti0713_DW.UnitDelay_DSTATE_m;

  /* Product: '<S45>/delta rise limit' incorporates:
   *  Constant: '<S35>/Constant2'
   *  SampleTimeMath: '<S45>/sample time'
   *
   * About '<S45>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Min1_j = 1.0;

  /* UnitDelay: '<S6>/Unit Delay7' */
  rtb_UnitDelay7 = VxSpdEsti0713_DW.UnitDelay7_DSTATE;

  /* Sum: '<S6>/Add3' incorporates:
   *  Inport: '<Root>/IMU_Ax_mpss'
   *  Product: '<S6>/Divide3'
   *  Sum: '<S6>/Add7'
   *  UnitDelay: '<S6>/Unit Delay3'
   *  UnitDelay: '<S6>/Unit Delay7'
   */
  VxSpdEsti0713_DW.UnitDelay7_DSTATE = (rtb_Add14 -
    VxSpdEsti0713_DW.UnitDelay3_DSTATE_o) / 0.0099999997764825821 -
    VxSpdEsti0713_U.IMU_Ax_mpss;

  /* Product: '<S6>/Divide7' incorporates:
   *  Sum: '<S6>/Add14'
   *  UnitDelay: '<S6>/Unit Delay7'
   */
  rtb_UnitDelay7 = (VxSpdEsti0713_DW.UnitDelay7_DSTATE - rtb_UnitDelay7) /
    0.0099999997764825821;

  /* Switch: '<S46>/Switch2' incorporates:
   *  Constant: '<S35>/Constant1'
   *  RelationalOperator: '<S46>/LowerRelop1'
   *  RelationalOperator: '<S46>/UpperRelop'
   *  Switch: '<S46>/Switch'
   */
  if (rtb_UnitDelay7 > 100.0) {
    rtb_UnitDelay7 = 100.0;
  } else if (rtb_UnitDelay7 < -100.0) {
    /* Switch: '<S46>/Switch' incorporates:
     *  Constant: '<S35>/Constant'
     */
    rtb_UnitDelay7 = -100.0;
  }

  /* Sum: '<S45>/Difference Inputs1' incorporates:
   *  Switch: '<S46>/Switch2'
   *  UnitDelay: '<S45>/Delay Input2'
   *
   * Block description for '<S45>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S45>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UnitDelay7 -= VxSpdEsti0713_DW.DelayInput2_DSTATE_o;

  /* Switch: '<S47>/Switch2' incorporates:
   *  RelationalOperator: '<S47>/LowerRelop1'
   */
  if (!(rtb_UnitDelay7 > 1.0)) {
    /* Switch: '<S47>/Switch' incorporates:
     *  RelationalOperator: '<S47>/UpperRelop'
     */
    if (rtb_UnitDelay7 < -1.0) {
      rtb_Min1_j = -1.0;
    } else {
      rtb_Min1_j = rtb_UnitDelay7;
    }

    /* End of Switch: '<S47>/Switch' */
  }

  /* End of Switch: '<S47>/Switch2' */

  /* Sum: '<S45>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S45>/Delay Input2'
   *
   * Block description for '<S45>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S45>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VxSpdEsti0713_DW.DelayInput2_DSTATE_o += rtb_Min1_j;

  /* Saturate: '<S35>/Saturation' incorporates:
   *  UnitDelay: '<S45>/Delay Input2'
   *
   * Block description for '<S45>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (VxSpdEsti0713_DW.DelayInput2_DSTATE_o > 0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_m = 0.5;
  } else if (VxSpdEsti0713_DW.DelayInput2_DSTATE_o < -0.5) {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_m = -0.5;
  } else {
    VxSpdEsti0713_DW.UnitDelay_DSTATE_m = VxSpdEsti0713_DW.DelayInput2_DSTATE_o;
  }

  /* End of Saturate: '<S35>/Saturation' */

  /* Logic: '<S2>/Logical Operator1' incorporates:
   *  Abs: '<S5>/Abs'
   *  Abs: '<S5>/Abs1'
   *  Abs: '<S5>/Abs2'
   *  Abs: '<S5>/Abs3'
   *  Abs: '<S6>/Abs'
   *  Abs: '<S6>/Abs1'
   *  Abs: '<S6>/Abs2'
   *  Abs: '<S6>/Abs3'
   *  Abs: '<S7>/Abs'
   *  Abs: '<S7>/Abs1'
   *  Abs: '<S7>/Abs2'
   *  Abs: '<S7>/Abs3'
   *  Constant: '<S10>/Constant'
   *  Constant: '<S11>/Constant'
   *  Constant: '<S28>/Constant'
   *  Constant: '<S29>/Constant'
   *  Constant: '<S30>/Constant'
   *  Constant: '<S31>/Constant'
   *  Constant: '<S48>/Constant'
   *  Constant: '<S49>/Constant'
   *  Constant: '<S50>/Constant'
   *  Constant: '<S51>/Constant'
   *  Constant: '<S8>/Constant'
   *  Constant: '<S9>/Constant'
   *  Gain: '<S12>/Gain1'
   *  Gain: '<S13>/Gain1'
   *  Gain: '<S14>/Gain1'
   *  Gain: '<S15>/Gain1'
   *  Gain: '<S32>/Gain1'
   *  Gain: '<S33>/Gain1'
   *  Gain: '<S34>/Gain1'
   *  Gain: '<S35>/Gain1'
   *  Inport: '<Root>/IMU_Ax_mpss'
   *  Logic: '<S2>/Logical Operator'
   *  RelationalOperator: '<S10>/Compare'
   *  RelationalOperator: '<S11>/Compare'
   *  RelationalOperator: '<S28>/Compare'
   *  RelationalOperator: '<S29>/Compare'
   *  RelationalOperator: '<S30>/Compare'
   *  RelationalOperator: '<S31>/Compare'
   *  RelationalOperator: '<S48>/Compare'
   *  RelationalOperator: '<S49>/Compare'
   *  RelationalOperator: '<S50>/Compare'
   *  RelationalOperator: '<S51>/Compare'
   *  RelationalOperator: '<S8>/Compare'
   *  RelationalOperator: '<S9>/Compare'
   *  Sum: '<S12>/Add'
   *  Sum: '<S13>/Add'
   *  Sum: '<S14>/Add'
   *  Sum: '<S15>/Add'
   *  Sum: '<S32>/Add'
   *  Sum: '<S33>/Add'
   *  Sum: '<S34>/Add'
   *  Sum: '<S35>/Add'
   *  Sum: '<S5>/Add'
   *  Sum: '<S5>/Add1'
   *  Sum: '<S5>/Add2'
   *  Sum: '<S5>/Add3'
   *  Sum: '<S7>/Add'
   *  Sum: '<S7>/Add1'
   *  Sum: '<S7>/Add2'
   *  Sum: '<S7>/Add3'
   *  UnitDelay: '<Root>/Unit Delay'
   *  UnitDelay: '<S12>/Unit Delay'
   *  UnitDelay: '<S13>/Unit Delay'
   *  UnitDelay: '<S14>/Unit Delay'
   *  UnitDelay: '<S15>/Unit Delay'
   *  UnitDelay: '<S32>/Unit Delay'
   *  UnitDelay: '<S33>/Unit Delay'
   *  UnitDelay: '<S34>/Unit Delay'
   *  UnitDelay: '<S35>/Unit Delay'
   */
  rtb_LogicalOperator1_l_idx_0 = ((fabs(rtb_VxFL_mps -
    VxSpdEsti0713_Y.VehVxEst_mps) <= 0.5) && ((fabs((0.7 *
    VxSpdEsti0713_DW.UnitDelay_DSTATE + rtb_Gain) - VxSpdEsti0713_U.IMU_Ax_mpss)
    <= 0.5) || (fabs(0.7 * VxSpdEsti0713_DW.UnitDelay_DSTATE_dh + rtb_Divide3) <=
                0.5)));
  rtb_LogicalOperator1_l_idx_1 = ((fabs(rtb_Divide3_b -
    VxSpdEsti0713_Y.VehVxEst_mps) <= 0.5) && ((fabs((0.7 *
    VxSpdEsti0713_DW.UnitDelay_DSTATE_k + rtb_Gain_g) -
    VxSpdEsti0713_U.IMU_Ax_mpss) <= 0.5) || (fabs(0.7 *
    VxSpdEsti0713_DW.UnitDelay_DSTATE_l + rtb_UnitDelay4) <= 0.5)));
  rtb_LogicalOperator1_l_idx_2 = ((fabs(rtb_Divide -
    VxSpdEsti0713_Y.VehVxEst_mps) <= 0.5) && ((fabs((0.7 *
    VxSpdEsti0713_DW.UnitDelay_DSTATE_h + rtb_Gain_gu) -
    VxSpdEsti0713_U.IMU_Ax_mpss) <= 0.5) || (fabs(0.7 *
    VxSpdEsti0713_DW.UnitDelay_DSTATE_p + rtb_UnitDelay5) <= 0.5)));
  rtb_LogicalOperator1_l_idx_3 = ((fabs(rtb_Add14 - VxSpdEsti0713_Y.VehVxEst_mps)
    <= 0.5) && ((fabs((0.7 * VxSpdEsti0713_DW.UnitDelay_DSTATE_j + rtb_Divide2)
                      - VxSpdEsti0713_U.IMU_Ax_mpss) <= 0.5) || (fabs(0.7 *
    VxSpdEsti0713_DW.UnitDelay_DSTATE_m + rtb_UnitDelay6) <= 0.5)));

  /* Switch: '<S4>/Switch1' incorporates:
   *  Constant: '<S4>/Constant'
   */
  if (rtb_LogicalOperator1_l_idx_0) {
    rtb_Gain_g = rtb_VxFL_mps;
  } else {
    rtb_Gain_g = (rtNaN);
  }

  /* End of Switch: '<S4>/Switch1' */

  /* Switch: '<S4>/Switch2' incorporates:
   *  Constant: '<S4>/Constant1'
   */
  if (rtb_LogicalOperator1_l_idx_1) {
    rtb_Min1_j = rtb_Divide3_b;
  } else {
    rtb_Min1_j = (rtNaN);
  }

  /* End of Switch: '<S4>/Switch2' */

  /* Switch: '<S4>/Switch3' incorporates:
   *  Constant: '<S4>/Constant2'
   */
  if (rtb_LogicalOperator1_l_idx_2) {
    rtb_Gain_gu = rtb_Divide;
  } else {
    rtb_Gain_gu = (rtNaN);
  }

  /* End of Switch: '<S4>/Switch3' */

  /* Switch: '<S4>/Switch4' incorporates:
   *  Constant: '<S4>/Constant3'
   */
  if (rtb_LogicalOperator1_l_idx_3) {
    rtb_Divide2 = rtb_Add14;
  } else {
    rtb_Divide2 = (rtNaN);
  }

  /* End of Switch: '<S4>/Switch4' */

  /* Product: '<S4>/Divide' incorporates:
   *  DataTypeConversion: '<S4>/Cast To Double'
   *  DataTypeConversion: '<S4>/Cast To Double1'
   *  DataTypeConversion: '<S4>/Cast To Double2'
   *  DataTypeConversion: '<S4>/Cast To Double3'
   *  Sum: '<S4>/Add'
   *  Sum: '<S4>/Add1'
   */
  rtb_Gain = (((rtb_Gain_g + rtb_Min1_j) + rtb_Gain_gu) + rtb_Divide2) /
    ((((real_T)rtb_LogicalOperator1_l_idx_0 + (real_T)
       rtb_LogicalOperator1_l_idx_1) + (real_T)rtb_LogicalOperator1_l_idx_2) +
     (real_T)rtb_LogicalOperator1_l_idx_3);

  /* DataTypeConversion: '<S4>/Cast To Double4' incorporates:
   *  Sum: '<S4>/Add2'
   *  UnitDelay: '<S4>/Unit Delay'
   */
  rtb_Add1_f = (real32_T)(rtb_Gain - VxSpdEsti0713_DW.UnitDelay_DSTATE_hv);

  /* Switch: '<S62>/Switch2' incorporates:
   *  Constant: '<S57>/Constant'
   *  Constant: '<S57>/Constant1'
   *  DataTypeConversion: '<S4>/Cast To Double4'
   *  RelationalOperator: '<S62>/LowerRelop1'
   *  RelationalOperator: '<S62>/UpperRelop'
   *  Switch: '<S62>/Switch'
   */
  if (rtb_Add1_f > 100.0F) {
    rtb_Add1_f = 100.0F;
  } else if (rtb_Add1_f < -100.0F) {
    /* Switch: '<S62>/Switch' incorporates:
     *  Constant: '<S57>/Constant'
     */
    rtb_Add1_f = -100.0F;
  }

  /* Sum: '<S61>/Difference Inputs1' incorporates:
   *  Switch: '<S62>/Switch2'
   *  UnitDelay: '<S61>/Delay Input2'
   *
   * Block description for '<S61>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S61>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add1_f -= VxSpdEsti0713_DW.DelayInput2_DSTATE_b;

  /* Product: '<S61>/delta rise limit' incorporates:
   *  Constant: '<S57>/Constant2'
   *  SampleTimeMath: '<S61>/sample time'
   *
   * About '<S61>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Switch2_k = 1.0F;

  /* Switch: '<S63>/Switch2' incorporates:
   *  RelationalOperator: '<S63>/LowerRelop1'
   */
  if (!(rtb_Add1_f > 1.0F)) {
    /* Switch: '<S63>/Switch' incorporates:
     *  RelationalOperator: '<S63>/UpperRelop'
     */
    if (rtb_Add1_f < -1.0F) {
      rtb_Switch2_k = -1.0F;
    } else {
      rtb_Switch2_k = rtb_Add1_f;
    }

    /* End of Switch: '<S63>/Switch' */
  }

  /* End of Switch: '<S63>/Switch2' */

  /* Sum: '<S61>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S61>/Delay Input2'
   *
   * Block description for '<S61>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S61>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VxSpdEsti0713_DW.DelayInput2_DSTATE_b += rtb_Switch2_k;

  /* Saturate: '<S57>/Saturation' incorporates:
   *  UnitDelay: '<S61>/Delay Input2'
   *
   * Block description for '<S61>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (VxSpdEsti0713_DW.DelayInput2_DSTATE_b > 0.5F) {
    rtb_Add1_f = 0.5F;
  } else if (VxSpdEsti0713_DW.DelayInput2_DSTATE_b < -0.5F) {
    rtb_Add1_f = -0.5F;
  } else {
    rtb_Add1_f = VxSpdEsti0713_DW.DelayInput2_DSTATE_b;
  }

  /* End of Saturate: '<S57>/Saturation' */

  /* Logic: '<S3>/Logical Operator2' incorporates:
   *  Constant: '<S52>/Constant'
   *  Delay: '<S3>/Delay'
   *  Inport: '<Root>/TrqReq_Nm'
   *  Logic: '<S3>/Logical Operator'
   *  Logic: '<S3>/Logical Operator1'
   *  RelationalOperator: '<S52>/Compare'
   *  Sum: '<S3>/Add1'
   */
  rtb_LogicalOperator2 = (((!rtb_LogicalOperator1_l_idx_0) &&
    (!rtb_LogicalOperator1_l_idx_1) && (!rtb_LogicalOperator1_l_idx_2) &&
    (!rtb_LogicalOperator1_l_idx_3)) || (VxSpdEsti0713_U.TrqReq_Nm -
    VxSpdEsti0713_DW.Delay_DSTATE[0] > 20.0F));

  /* Switch: '<S3>/Switch1' */
  if (rtb_LogicalOperator2) {
    /* Saturate: '<S3>/Saturation1' incorporates:
     *  Constant: '<S3>/steptime1'
     *  Inport: '<Root>/IMU_Ax_mpss'
     *  Product: '<S3>/Product'
     *  Sum: '<S3>/Add'
     *  UnitDelay: '<S3>/Unit Delay'
     */
    if (VxSpdEsti0713_DW.UnitDelay_DSTATE_pp > 50.0F) {
      VxSpdEsti0713_DW.UnitDelay_DSTATE_pp = 50.0F;
    } else if (VxSpdEsti0713_DW.UnitDelay_DSTATE_pp < -10.0F) {
      VxSpdEsti0713_DW.UnitDelay_DSTATE_pp = -10.0F;
    }

    VxSpdEsti0713_DW.UnitDelay_DSTATE_pp += VxSpdEsti0713_U.IMU_Ax_mpss * 0.01F;
  } else {
    /* Saturate: '<S3>/Saturation1' incorporates:
     *  Constant: '<S3>/Constant'
     *  UnitDelay: '<S3>/Unit Delay'
     */
    VxSpdEsti0713_DW.UnitDelay_DSTATE_pp = 0.0F;
  }

  /* End of Switch: '<S3>/Switch1' */

  /* Switch: '<S3>/Switch3' incorporates:
   *  Logic: '<S3>/Logical Operator3'
   *  UnitDelay: '<S3>/Unit Delay3'
   */
  if (!VxSpdEsti0713_DW.UnitDelay3_DSTATE_j) {
    /* Switch: '<S3>/Switch4' incorporates:
     *  Constant: '<S3>/steptime3'
     *  UnitDelay: '<Root>/Unit Delay1'
     *  UnitDelay: '<S3>/Unit Delay4'
     */
    if (rtb_LogicalOperator2) {
      VxSpdEsti0713_DW.UnitDelay4_DSTATE_d = VxSpdEsti0713_Y.VehVxEst_mps;
    } else {
      VxSpdEsti0713_DW.UnitDelay4_DSTATE_d = 0.0F;
    }

    /* End of Switch: '<S3>/Switch4' */
  }

  /* End of Switch: '<S3>/Switch3' */

  /* Saturate: '<S3>/Saturation' incorporates:
   *  UnitDelay: '<S3>/Unit Delay'
   */
  if (VxSpdEsti0713_DW.UnitDelay_DSTATE_pp > 50.0F) {
    rtb_UkYk1_nb = 50.0F;
  } else if (VxSpdEsti0713_DW.UnitDelay_DSTATE_pp < -10.0F) {
    rtb_UkYk1_nb = -10.0F;
  } else {
    rtb_UkYk1_nb = VxSpdEsti0713_DW.UnitDelay_DSTATE_pp;
  }

  /* Sum: '<S3>/Add3' incorporates:
   *  Saturate: '<S3>/Saturation'
   *  UnitDelay: '<S3>/Unit Delay4'
   */
  rtb_Switch2_k = rtb_UkYk1_nb + VxSpdEsti0713_DW.UnitDelay4_DSTATE_d;

  /* Switch: '<S3>/Switch6' incorporates:
   *  Constant: '<S3>/Reset'
   *  Constant: '<S3>/Steptime'
   *  Constant: '<S53>/Constant'
   *  MinMax: '<S2>/Min1'
   *  RelationalOperator: '<S53>/Compare'
   *  Sum: '<S3>/Add10'
   *  Sum: '<S3>/Add2'
   *  UnitDelay: '<S3>/Unit Delay2'
   */
  if (fmax(fmax(fmax(rtb_VxFL_mps, rtb_Divide3_b), rtb_Divide), rtb_Add14) -
      rtb_Switch2_k <= 0.0) {
    rtb_UkYk1_nb = VxSpdEsti0713_DW.UnitDelay2_DSTATE_h + 0.01F;
  } else {
    rtb_UkYk1_nb = 0.0F;
  }

  /* MinMax: '<S3>/Min' incorporates:
   *  Constant: '<S3>/ResetDelay'
   *  Switch: '<S3>/Switch6'
   *  UnitDelay: '<S3>/Unit Delay2'
   */
  VxSpdEsti0713_DW.UnitDelay2_DSTATE_h = fminf(rtb_UkYk1_nb, 0.1F);

  /* Switch: '<S4>/Switch' incorporates:
   *  Constant: '<S3>/ResetDelay'
   *  Logic: '<S4>/Logical Operator'
   *  Logic: '<S4>/Logical Operator1'
   *  RelationalOperator: '<S3>/Relational Operator9'
   *  UnitDelay: '<S3>/Unit Delay2'
   */
  if (rtb_LogicalOperator1_l_idx_0 || rtb_LogicalOperator1_l_idx_1 ||
      rtb_LogicalOperator1_l_idx_2 || rtb_LogicalOperator1_l_idx_3 ||
      (VxSpdEsti0713_DW.UnitDelay2_DSTATE_h >= 0.1F)) {
    /* Sum: '<S57>/Add' incorporates:
     *  Gain: '<S57>/Gain'
     *  Gain: '<S57>/Gain1'
     *  UnitDelay: '<S57>/Unit Delay'
     */
    rtb_Switch2_k = 0.3F * VxSpdEsti0713_DW.UnitDelay_DSTATE_pz + 0.7F *
      rtb_Add1_f;

    /* Switch: '<S4>/Switch6' incorporates:
     *  Constant: '<S55>/Constant'
     *  Logic: '<S4>/Logical Operator2'
     *  MinMax: '<S4>/Min'
     *  RelationalOperator: '<S55>/Compare'
     */
    if (!(rtb_Switch2_k > 0.1F)) {
      /* Switch: '<S4>/Switch5' incorporates:
       *  Constant: '<S54>/Constant'
       *  MinMax: '<S4>/Min1'
       *  RelationalOperator: '<S54>/Compare'
       */
      if (rtb_Switch2_k < -0.1F) {
        rtb_Switch2_k = (real32_T)fmax(fmax(fmax(rtb_Gain_g, rtb_Min1_j),
          rtb_Gain_gu), rtb_Divide2);
      } else {
        rtb_Switch2_k = (real32_T)rtb_Gain;
      }

      /* End of Switch: '<S4>/Switch5' */
    } else {
      rtb_Switch2_k = (real32_T)fmin(fmin(fmin(rtb_Gain_g, rtb_Min1_j),
        rtb_Gain_gu), rtb_Divide2);
    }

    /* End of Switch: '<S4>/Switch6' */
  }

  /* End of Switch: '<S4>/Switch' */

  /* Switch: '<S59>/Switch2' incorporates:
   *  Constant: '<S56>/Constant'
   *  Constant: '<S56>/Constant1'
   *  RelationalOperator: '<S59>/LowerRelop1'
   *  RelationalOperator: '<S59>/UpperRelop'
   *  Switch: '<S59>/Switch'
   */
  if (rtb_Switch2_k > 100.0F) {
    rtb_Switch2_k = 100.0F;
  } else if (rtb_Switch2_k < -100.0F) {
    /* Switch: '<S59>/Switch' incorporates:
     *  Constant: '<S56>/Constant'
     */
    rtb_Switch2_k = -100.0F;
  }

  /* Sum: '<S58>/Difference Inputs1' incorporates:
   *  Switch: '<S59>/Switch2'
   *  UnitDelay: '<S58>/Delay Input2'
   *
   * Block description for '<S58>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S58>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_UkYk1_nb = rtb_Switch2_k - VxSpdEsti0713_DW.DelayInput2_DSTATE_c4;

  /* Product: '<S58>/delta rise limit' incorporates:
   *  Constant: '<S56>/Constant2'
   *  SampleTimeMath: '<S58>/sample time'
   *
   * About '<S58>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Switch2_k = 1.0F;

  /* Switch: '<S60>/Switch2' incorporates:
   *  RelationalOperator: '<S60>/LowerRelop1'
   */
  if (!(rtb_UkYk1_nb > 1.0F)) {
    /* Switch: '<S60>/Switch' incorporates:
     *  RelationalOperator: '<S60>/UpperRelop'
     */
    if (rtb_UkYk1_nb < -1.0F) {
      rtb_Switch2_k = -1.0F;
    } else {
      rtb_Switch2_k = rtb_UkYk1_nb;
    }

    /* End of Switch: '<S60>/Switch' */
  }

  /* End of Switch: '<S60>/Switch2' */

  /* Sum: '<S58>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S58>/Delay Input2'
   *
   * Block description for '<S58>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S58>/Delay Input2':
   *
   *  Store in Global RAM
   */
  VxSpdEsti0713_DW.DelayInput2_DSTATE_c4 += rtb_Switch2_k;

  /* Saturate: '<S56>/Saturation' incorporates:
   *  UnitDelay: '<S58>/Delay Input2'
   *
   * Block description for '<S58>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (VxSpdEsti0713_DW.DelayInput2_DSTATE_c4 > 0.5F) {
    rtb_Switch2_k = 0.5F;
  } else if (VxSpdEsti0713_DW.DelayInput2_DSTATE_c4 < -0.5F) {
    rtb_Switch2_k = -0.5F;
  } else {
    rtb_Switch2_k = VxSpdEsti0713_DW.DelayInput2_DSTATE_c4;
  }

  /* End of Saturate: '<S56>/Saturation' */

  /* Sum: '<S56>/Add' incorporates:
   *  Gain: '<S56>/Gain'
   *  Gain: '<S56>/Gain1'
   *  UnitDelay: '<Root>/Unit Delay1'
   *  UnitDelay: '<S56>/Unit Delay'
   */
  VxSpdEsti0713_Y.VehVxEst_mps = 0.3F * VxSpdEsti0713_DW.UnitDelay_DSTATE_o +
    0.7F * rtb_Switch2_k;

  /* Update for UnitDelay: '<S5>/Unit Delay' */
  VxSpdEsti0713_DW.UnitDelay_DSTATE_d = rtb_VxFL_mps;

  /* Update for UnitDelay: '<S5>/Unit Delay1' */
  VxSpdEsti0713_DW.UnitDelay1_DSTATE = rtb_Divide3_b;

  /* Update for UnitDelay: '<S5>/Unit Delay2' */
  VxSpdEsti0713_DW.UnitDelay2_DSTATE = rtb_Divide;

  /* Update for UnitDelay: '<S5>/Unit Delay3' */
  VxSpdEsti0713_DW.UnitDelay3_DSTATE = rtb_Add14;

  /* Update for UnitDelay: '<S6>/Unit Delay' */
  VxSpdEsti0713_DW.UnitDelay_DSTATE_kf = rtb_VxFL_mps;

  /* Update for UnitDelay: '<S6>/Unit Delay1' */
  VxSpdEsti0713_DW.UnitDelay1_DSTATE_k = rtb_Divide3_b;

  /* Update for UnitDelay: '<S6>/Unit Delay2' */
  VxSpdEsti0713_DW.UnitDelay2_DSTATE_p = rtb_Divide;

  /* Update for UnitDelay: '<S6>/Unit Delay3' */
  VxSpdEsti0713_DW.UnitDelay3_DSTATE_o = rtb_Add14;

  /* Update for UnitDelay: '<S4>/Unit Delay' */
  VxSpdEsti0713_DW.UnitDelay_DSTATE_hv = rtb_Gain;

  /* Update for UnitDelay: '<S57>/Unit Delay' */
  VxSpdEsti0713_DW.UnitDelay_DSTATE_pz = rtb_Add1_f;

  /* Update for Delay: '<S3>/Delay' incorporates:
   *  Inport: '<Root>/TrqReq_Nm'
   */
  VxSpdEsti0713_DW.Delay_DSTATE[0] = VxSpdEsti0713_DW.Delay_DSTATE[1];
  VxSpdEsti0713_DW.Delay_DSTATE[1] = VxSpdEsti0713_DW.Delay_DSTATE[2];
  VxSpdEsti0713_DW.Delay_DSTATE[2] = VxSpdEsti0713_DW.Delay_DSTATE[3];
  VxSpdEsti0713_DW.Delay_DSTATE[3] = VxSpdEsti0713_DW.Delay_DSTATE[4];
  VxSpdEsti0713_DW.Delay_DSTATE[4] = VxSpdEsti0713_U.TrqReq_Nm;

  /* Update for UnitDelay: '<S3>/Unit Delay3' */
  VxSpdEsti0713_DW.UnitDelay3_DSTATE_j = rtb_LogicalOperator2;

  /* Update for UnitDelay: '<S56>/Unit Delay' */
  VxSpdEsti0713_DW.UnitDelay_DSTATE_o = rtb_Switch2_k;
}

/* Model initialize function */
void VxSpdEsti0713_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));
}

/* Model terminate function */
void VxSpdEsti0713_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
