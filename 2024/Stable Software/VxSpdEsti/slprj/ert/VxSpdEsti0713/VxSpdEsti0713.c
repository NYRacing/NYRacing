/*
 * File: VxSpdEsti0713.c
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

#include "VxSpdEsti0713.h"
#include "rtwtypes.h"
#include <math.h>
#include "VxSpdEsti0713_private.h"
#include "rt_nonfinite.h"

/* Output and update for referenced model: 'VxSpdEsti0713' */
void VxSpdEsti0713(const real32_T *rtu_IMU_Ax_mpss, const real32_T
                   *rtu_IMU_YawRate_dps, const real32_T *rtu_WhlStrAng_deg,
                   const real32_T *rtu_TrqReq_Nm, const real32_T
                   *rtu_WhlSpdFL_mps, const real32_T *rtu_WhlSpdFR_mps, const
                   real32_T *rtu_WhlSpdRL_mps, const real32_T *rtu_WhlSpdRR_mps,
                   real32_T *rty_VehVxEst_mps, DW_VxSpdEsti0713_f_T *localDW)
{
  real_T rtb_Add14;
  real_T rtb_Add7;
  real_T rtb_Divide;
  real_T rtb_Divide1;
  real_T rtb_Gain;
  real_T rtb_Gain_e;
  real_T rtb_Gain_g;
  real_T rtb_Gain_k;
  real_T rtb_Gain_mj;
  real_T rtb_Saturation_gy;
  real_T rtb_Switch2_b;
  real_T rtb_Switch2_b2;
  real_T rtb_Switch2_ct;
  real_T rtb_Switch2_hz;
  real_T rtb_Switch2_j3;
  real_T rtb_VxFL_mps;
  real32_T rtb_Add3_c;
  real32_T rtb_Product1;
  real32_T rtb_UkYk1_de;
  real32_T rtb_UnitDelay_a;
  boolean_T rtb_LogicalOperator1_l_idx_0;
  boolean_T rtb_LogicalOperator1_l_idx_1;
  boolean_T rtb_LogicalOperator1_l_idx_2;
  boolean_T rtb_LogicalOperator1_l_idx_3;
  boolean_T rtb_LogicalOperator2;

  /* Gain: '<S12>/Gain' incorporates:
   *  UnitDelay: '<S12>/Unit Delay'
   */
  rtb_Gain = 0.3 * localDW->UnitDelay_DSTATE;

  /* UnitDelay: '<S16>/Delay Input2' incorporates:
   *  UnitDelay: '<S12>/Unit Delay'
   *
   * Block description for '<S16>/Delay Input2':
   *
   *  Store in Global RAM
   */
  localDW->UnitDelay_DSTATE = localDW->DelayInput2_DSTATE;

  /* Product: '<S16>/delta rise limit' incorporates:
   *  Constant: '<S12>/Constant2'
   *  SampleTimeMath: '<S16>/sample time'
   *
   * About '<S16>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Add7 = 1.0;

  /* Trigonometry: '<S1>/Cos' */
  rtb_Add3_c = cosf(*rtu_WhlStrAng_deg);

  /* Product: '<S1>/Product1' */
  rtb_Product1 = *rtu_WhlSpdFL_mps * rtb_Add3_c;

  /* Product: '<S1>/Product3' */
  rtb_Add14 = VxSpdEsti0713_ConstB.Gain1 * *rtu_IMU_YawRate_dps;

  /* Sum: '<S1>/Add2' */
  rtb_VxFL_mps = rtb_Product1 - rtb_Add14;

  /* Product: '<S5>/Divide' incorporates:
   *  Sum: '<S5>/Add4'
   *  UnitDelay: '<S5>/Unit Delay'
   */
  rtb_Divide = (rtb_VxFL_mps - localDW->UnitDelay_DSTATE_d) /
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
   *  UnitDelay: '<S12>/Unit Delay'
   *
   * Block description for '<S16>/Difference Inputs1':
   *
   *  Add in CPU
   */
  rtb_Divide -= localDW->UnitDelay_DSTATE;

  /* Switch: '<S18>/Switch2' incorporates:
   *  RelationalOperator: '<S18>/LowerRelop1'
   */
  if (!(rtb_Divide > 1.0)) {
    /* Switch: '<S18>/Switch' incorporates:
     *  RelationalOperator: '<S18>/UpperRelop'
     */
    if (rtb_Divide < -1.0) {
      rtb_Add7 = -1.0;
    } else {
      rtb_Add7 = rtb_Divide;
    }

    /* End of Switch: '<S18>/Switch' */
  }

  /* End of Switch: '<S18>/Switch2' */

  /* Sum: '<S16>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S12>/Unit Delay'
   *
   * Block description for '<S16>/Difference Inputs2':
   *
   *  Add in CPU
   */
  localDW->DelayInput2_DSTATE = rtb_Add7 + localDW->UnitDelay_DSTATE;

  /* Saturate: '<S12>/Saturation' */
  if (localDW->DelayInput2_DSTATE > 0.5) {
    rtb_Divide = 0.5;
  } else if (localDW->DelayInput2_DSTATE < -0.5) {
    rtb_Divide = -0.5;
  } else {
    rtb_Divide = localDW->DelayInput2_DSTATE;
  }

  /* End of Saturate: '<S12>/Saturation' */

  /* Sum: '<S5>/Add' incorporates:
   *  Gain: '<S12>/Gain1'
   *  Sum: '<S12>/Add'
   */
  rtb_Switch2_b = (0.7 * rtb_Divide + rtb_Gain) - *rtu_IMU_Ax_mpss;

  /* Abs: '<S5>/Abs' */
  rtb_Switch2_ct = fabs(rtb_Switch2_b);

  /* Gain: '<S13>/Gain' incorporates:
   *  UnitDelay: '<S13>/Unit Delay'
   */
  rtb_Gain_g = 0.3 * localDW->UnitDelay_DSTATE_k;

  /* Product: '<S19>/delta rise limit' incorporates:
   *  Constant: '<S13>/Constant2'
   *  SampleTimeMath: '<S19>/sample time'
   *
   * About '<S19>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Add7 = 1.0;

  /* Product: '<S1>/Product2' */
  rtb_Add3_c *= *rtu_WhlSpdFR_mps;

  /* Sum: '<S1>/Add3' */
  rtb_Gain = rtb_Add14 + rtb_Add3_c;

  /* Product: '<S5>/Divide1' incorporates:
   *  Sum: '<S5>/Add5'
   *  UnitDelay: '<S5>/Unit Delay1'
   */
  rtb_Divide1 = (rtb_Gain - localDW->UnitDelay1_DSTATE) / 0.0099999997764825821;

  /* Switch: '<S20>/Switch2' incorporates:
   *  Constant: '<S13>/Constant1'
   *  RelationalOperator: '<S20>/LowerRelop1'
   *  RelationalOperator: '<S20>/UpperRelop'
   *  Switch: '<S20>/Switch'
   */
  if (rtb_Divide1 > 100.0) {
    rtb_Divide1 = 100.0;
  } else if (rtb_Divide1 < -100.0) {
    /* Switch: '<S20>/Switch' incorporates:
     *  Constant: '<S13>/Constant'
     */
    rtb_Divide1 = -100.0;
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
  rtb_Divide1 -= localDW->DelayInput2_DSTATE_c;

  /* Switch: '<S21>/Switch2' incorporates:
   *  RelationalOperator: '<S21>/LowerRelop1'
   */
  if (!(rtb_Divide1 > 1.0)) {
    /* Switch: '<S21>/Switch' incorporates:
     *  RelationalOperator: '<S21>/UpperRelop'
     */
    if (rtb_Divide1 < -1.0) {
      rtb_Add7 = -1.0;
    } else {
      rtb_Add7 = rtb_Divide1;
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
  localDW->DelayInput2_DSTATE_c += rtb_Add7;

  /* Saturate: '<S13>/Saturation' incorporates:
   *  UnitDelay: '<S19>/Delay Input2'
   *
   * Block description for '<S19>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (localDW->DelayInput2_DSTATE_c > 0.5) {
    localDW->UnitDelay_DSTATE_k = 0.5;
  } else if (localDW->DelayInput2_DSTATE_c < -0.5) {
    localDW->UnitDelay_DSTATE_k = -0.5;
  } else {
    localDW->UnitDelay_DSTATE_k = localDW->DelayInput2_DSTATE_c;
  }

  /* End of Saturate: '<S13>/Saturation' */

  /* Sum: '<S5>/Add1' incorporates:
   *  Gain: '<S13>/Gain1'
   *  Sum: '<S13>/Add'
   *  UnitDelay: '<S13>/Unit Delay'
   */
  rtb_Switch2_b = (0.7 * localDW->UnitDelay_DSTATE_k + rtb_Gain_g) -
    *rtu_IMU_Ax_mpss;

  /* Abs: '<S5>/Abs1' */
  rtb_Switch2_hz = fabs(rtb_Switch2_b);

  /* Product: '<S22>/delta rise limit' incorporates:
   *  Constant: '<S14>/Constant2'
   *  SampleTimeMath: '<S22>/sample time'
   *
   * About '<S22>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Add14 = 1.0;

  /* Product: '<S1>/Product' */
  rtb_Add7 = *rtu_IMU_YawRate_dps * VxSpdEsti0713_ConstB.Gain;

  /* Sum: '<S1>/Add' */
  rtb_Gain_g = *rtu_WhlSpdRL_mps - rtb_Add7;

  /* UnitDelay: '<S5>/Unit Delay2' incorporates:
   *  UnitDelay: '<S12>/Unit Delay'
   */
  localDW->UnitDelay_DSTATE = localDW->UnitDelay2_DSTATE;

  /* Sum: '<S5>/Add6' incorporates:
   *  UnitDelay: '<S12>/Unit Delay'
   */
  localDW->UnitDelay_DSTATE = rtb_Gain_g - localDW->UnitDelay_DSTATE;

  /* Product: '<S5>/Divide2' incorporates:
   *  UnitDelay: '<S12>/Unit Delay'
   */
  rtb_Divide1 = localDW->UnitDelay_DSTATE / 0.0099999997764825821;

  /* Switch: '<S23>/Switch2' incorporates:
   *  Constant: '<S14>/Constant1'
   *  RelationalOperator: '<S23>/LowerRelop1'
   *  RelationalOperator: '<S23>/UpperRelop'
   *  Switch: '<S23>/Switch'
   */
  if (rtb_Divide1 > 100.0) {
    rtb_Divide1 = 100.0;
  } else if (rtb_Divide1 < -100.0) {
    /* Switch: '<S23>/Switch' incorporates:
     *  Constant: '<S14>/Constant'
     */
    rtb_Divide1 = -100.0;
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
  rtb_Divide1 -= localDW->DelayInput2_DSTATE_m;

  /* Switch: '<S24>/Switch2' incorporates:
   *  RelationalOperator: '<S24>/LowerRelop1'
   */
  if (!(rtb_Divide1 > 1.0)) {
    /* Switch: '<S24>/Switch' incorporates:
     *  RelationalOperator: '<S24>/UpperRelop'
     */
    if (rtb_Divide1 < -1.0) {
      rtb_Add14 = -1.0;
    } else {
      rtb_Add14 = rtb_Divide1;
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
  localDW->DelayInput2_DSTATE_m += rtb_Add14;

  /* Saturate: '<S14>/Saturation' */
  if (localDW->DelayInput2_DSTATE_m > 0.5) {
    rtb_Saturation_gy = 0.5;
  } else if (localDW->DelayInput2_DSTATE_m < -0.5) {
    rtb_Saturation_gy = -0.5;
  } else {
    rtb_Saturation_gy = localDW->DelayInput2_DSTATE_m;
  }

  /* End of Saturate: '<S14>/Saturation' */

  /* Sum: '<S5>/Add2' incorporates:
   *  Gain: '<S14>/Gain'
   *  Gain: '<S14>/Gain1'
   *  Sum: '<S14>/Add'
   *  UnitDelay: '<S14>/Unit Delay'
   */
  rtb_Switch2_b = (0.3 * localDW->UnitDelay_DSTATE_h + 0.7 * rtb_Saturation_gy)
    - *rtu_IMU_Ax_mpss;

  /* Abs: '<S5>/Abs2' */
  rtb_Switch2_b2 = fabs(rtb_Switch2_b);

  /* Gain: '<S15>/Gain' incorporates:
   *  UnitDelay: '<S15>/Unit Delay'
   */
  rtb_Switch2_b = 0.3 * localDW->UnitDelay_DSTATE_j;

  /* Product: '<S25>/delta rise limit' incorporates:
   *  Constant: '<S15>/Constant2'
   *  SampleTimeMath: '<S25>/sample time'
   *
   * About '<S25>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Add14 = 1.0;

  /* Sum: '<S1>/Add1' */
  rtb_Divide1 = rtb_Add7 + *rtu_WhlSpdRR_mps;

  /* Product: '<S5>/Divide3' incorporates:
   *  Sum: '<S5>/Add7'
   *  UnitDelay: '<S5>/Unit Delay3'
   */
  rtb_Add7 = (rtb_Divide1 - localDW->UnitDelay3_DSTATE) / 0.0099999997764825821;

  /* Switch: '<S26>/Switch2' incorporates:
   *  Constant: '<S15>/Constant1'
   *  RelationalOperator: '<S26>/LowerRelop1'
   *  RelationalOperator: '<S26>/UpperRelop'
   *  Switch: '<S26>/Switch'
   */
  if (rtb_Add7 > 100.0) {
    rtb_Add7 = 100.0;
  } else if (rtb_Add7 < -100.0) {
    /* Switch: '<S26>/Switch' incorporates:
     *  Constant: '<S15>/Constant'
     */
    rtb_Add7 = -100.0;
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
  rtb_Add7 -= localDW->DelayInput2_DSTATE_e;

  /* Switch: '<S27>/Switch2' incorporates:
   *  RelationalOperator: '<S27>/LowerRelop1'
   */
  if (!(rtb_Add7 > 1.0)) {
    /* Switch: '<S27>/Switch' incorporates:
     *  RelationalOperator: '<S27>/UpperRelop'
     */
    if (rtb_Add7 < -1.0) {
      rtb_Add14 = -1.0;
    } else {
      rtb_Add14 = rtb_Add7;
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
  localDW->DelayInput2_DSTATE_e += rtb_Add14;

  /* Saturate: '<S15>/Saturation' incorporates:
   *  UnitDelay: '<S25>/Delay Input2'
   *
   * Block description for '<S25>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (localDW->DelayInput2_DSTATE_e > 0.5) {
    localDW->UnitDelay_DSTATE_j = 0.5;
  } else if (localDW->DelayInput2_DSTATE_e < -0.5) {
    localDW->UnitDelay_DSTATE_j = -0.5;
  } else {
    localDW->UnitDelay_DSTATE_j = localDW->DelayInput2_DSTATE_e;
  }

  /* End of Saturate: '<S15>/Saturation' */

  /* Sum: '<S5>/Add3' incorporates:
   *  Gain: '<S15>/Gain1'
   *  Sum: '<S15>/Add'
   *  UnitDelay: '<S15>/Unit Delay'
   */
  rtb_Switch2_j3 = (0.7 * localDW->UnitDelay_DSTATE_j + rtb_Switch2_b) -
    *rtu_IMU_Ax_mpss;

  /* Product: '<S36>/delta rise limit' incorporates:
   *  Constant: '<S32>/Constant2'
   *  SampleTimeMath: '<S36>/sample time'
   *
   * About '<S36>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Add14 = 1.0;

  /* UnitDelay: '<S6>/Unit Delay4' */
  rtb_Add7 = localDW->UnitDelay4_DSTATE;

  /* UnitDelay: '<S6>/Unit Delay' incorporates:
   *  UnitDelay: '<S12>/Unit Delay'
   */
  localDW->UnitDelay_DSTATE = localDW->UnitDelay_DSTATE_kf;

  /* Sum: '<S6>/Add4' incorporates:
   *  UnitDelay: '<S12>/Unit Delay'
   */
  localDW->UnitDelay_DSTATE = rtb_VxFL_mps - localDW->UnitDelay_DSTATE;

  /* Sum: '<S6>/Add' incorporates:
   *  Product: '<S6>/Divide'
   *  UnitDelay: '<S12>/Unit Delay'
   *  UnitDelay: '<S6>/Unit Delay4'
   */
  localDW->UnitDelay4_DSTATE = localDW->UnitDelay_DSTATE / 0.0099999997764825821
    - *rtu_IMU_Ax_mpss;

  /* Product: '<S6>/Divide4' incorporates:
   *  Sum: '<S6>/Add8'
   *  UnitDelay: '<S6>/Unit Delay4'
   */
  rtb_Add7 = (localDW->UnitDelay4_DSTATE - rtb_Add7) / 0.0099999997764825821;

  /* Switch: '<S37>/Switch2' incorporates:
   *  Constant: '<S32>/Constant1'
   *  RelationalOperator: '<S37>/LowerRelop1'
   *  RelationalOperator: '<S37>/UpperRelop'
   *  Switch: '<S37>/Switch'
   */
  if (rtb_Add7 > 100.0) {
    rtb_Add7 = 100.0;
  } else if (rtb_Add7 < -100.0) {
    /* Switch: '<S37>/Switch' incorporates:
     *  Constant: '<S32>/Constant'
     */
    rtb_Add7 = -100.0;
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
  rtb_Add7 -= localDW->DelayInput2_DSTATE_l;

  /* Switch: '<S38>/Switch2' incorporates:
   *  RelationalOperator: '<S38>/LowerRelop1'
   */
  if (!(rtb_Add7 > 1.0)) {
    /* Switch: '<S38>/Switch' incorporates:
     *  RelationalOperator: '<S38>/UpperRelop'
     */
    if (rtb_Add7 < -1.0) {
      rtb_Add14 = -1.0;
    } else {
      rtb_Add14 = rtb_Add7;
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
  localDW->DelayInput2_DSTATE_l += rtb_Add14;

  /* Saturate: '<S32>/Saturation' */
  if (localDW->DelayInput2_DSTATE_l > 0.5) {
    rtb_Add7 = 0.5;
  } else if (localDW->DelayInput2_DSTATE_l < -0.5) {
    rtb_Add7 = -0.5;
  } else {
    rtb_Add7 = localDW->DelayInput2_DSTATE_l;
  }

  /* End of Saturate: '<S32>/Saturation' */

  /* Gain: '<S33>/Gain' incorporates:
   *  UnitDelay: '<S33>/Unit Delay'
   */
  rtb_Gain_e = 0.3 * localDW->UnitDelay_DSTATE_l;

  /* Product: '<S39>/delta rise limit' incorporates:
   *  Constant: '<S33>/Constant2'
   *  SampleTimeMath: '<S39>/sample time'
   *
   * About '<S39>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Switch2_b = 1.0;

  /* UnitDelay: '<S6>/Unit Delay5' */
  rtb_Add14 = localDW->UnitDelay5_DSTATE;

  /* Product: '<S6>/Divide1' incorporates:
   *  Sum: '<S6>/Add5'
   *  UnitDelay: '<S12>/Unit Delay'
   *  UnitDelay: '<S6>/Unit Delay1'
   */
  localDW->UnitDelay_DSTATE = (rtb_Gain - localDW->UnitDelay1_DSTATE_k) /
    0.0099999997764825821;

  /* Sum: '<S6>/Add1' incorporates:
   *  UnitDelay: '<S12>/Unit Delay'
   *  UnitDelay: '<S6>/Unit Delay5'
   */
  localDW->UnitDelay5_DSTATE = localDW->UnitDelay_DSTATE - *rtu_IMU_Ax_mpss;

  /* Product: '<S6>/Divide5' incorporates:
   *  Sum: '<S6>/Add10'
   *  UnitDelay: '<S6>/Unit Delay5'
   */
  rtb_Add14 = (localDW->UnitDelay5_DSTATE - rtb_Add14) / 0.0099999997764825821;

  /* Switch: '<S40>/Switch2' incorporates:
   *  Constant: '<S33>/Constant1'
   *  RelationalOperator: '<S40>/LowerRelop1'
   *  RelationalOperator: '<S40>/UpperRelop'
   *  Switch: '<S40>/Switch'
   */
  if (rtb_Add14 > 100.0) {
    rtb_Add14 = 100.0;
  } else if (rtb_Add14 < -100.0) {
    /* Switch: '<S40>/Switch' incorporates:
     *  Constant: '<S33>/Constant'
     */
    rtb_Add14 = -100.0;
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
  rtb_Add14 -= localDW->DelayInput2_DSTATE_lp;

  /* Switch: '<S41>/Switch2' incorporates:
   *  RelationalOperator: '<S41>/LowerRelop1'
   */
  if (!(rtb_Add14 > 1.0)) {
    /* Switch: '<S41>/Switch' incorporates:
     *  RelationalOperator: '<S41>/UpperRelop'
     */
    if (rtb_Add14 < -1.0) {
      rtb_Switch2_b = -1.0;
    } else {
      rtb_Switch2_b = rtb_Add14;
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
  localDW->DelayInput2_DSTATE_lp += rtb_Switch2_b;

  /* Saturate: '<S33>/Saturation' incorporates:
   *  UnitDelay: '<S39>/Delay Input2'
   *
   * Block description for '<S39>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (localDW->DelayInput2_DSTATE_lp > 0.5) {
    localDW->UnitDelay_DSTATE_l = 0.5;
  } else if (localDW->DelayInput2_DSTATE_lp < -0.5) {
    localDW->UnitDelay_DSTATE_l = -0.5;
  } else {
    localDW->UnitDelay_DSTATE_l = localDW->DelayInput2_DSTATE_lp;
  }

  /* End of Saturate: '<S33>/Saturation' */

  /* Gain: '<S34>/Gain' incorporates:
   *  UnitDelay: '<S34>/Unit Delay'
   */
  rtb_Gain_k = 0.3 * localDW->UnitDelay_DSTATE_p;

  /* Product: '<S42>/delta rise limit' incorporates:
   *  Constant: '<S34>/Constant2'
   *  SampleTimeMath: '<S42>/sample time'
   *
   * About '<S42>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Switch2_b = 1.0;

  /* UnitDelay: '<S6>/Unit Delay6' */
  rtb_Add14 = localDW->UnitDelay6_DSTATE;

  /* Product: '<S6>/Divide2' incorporates:
   *  Sum: '<S6>/Add6'
   *  UnitDelay: '<S12>/Unit Delay'
   *  UnitDelay: '<S6>/Unit Delay2'
   */
  localDW->UnitDelay_DSTATE = (rtb_Gain_g - localDW->UnitDelay2_DSTATE_p) /
    0.0099999997764825821;

  /* Sum: '<S6>/Add2' incorporates:
   *  UnitDelay: '<S12>/Unit Delay'
   *  UnitDelay: '<S6>/Unit Delay6'
   */
  localDW->UnitDelay6_DSTATE = localDW->UnitDelay_DSTATE - *rtu_IMU_Ax_mpss;

  /* Product: '<S6>/Divide6' incorporates:
   *  Sum: '<S6>/Add12'
   *  UnitDelay: '<S6>/Unit Delay6'
   */
  rtb_Add14 = (localDW->UnitDelay6_DSTATE - rtb_Add14) / 0.0099999997764825821;

  /* Switch: '<S43>/Switch2' incorporates:
   *  Constant: '<S34>/Constant1'
   *  RelationalOperator: '<S43>/LowerRelop1'
   *  RelationalOperator: '<S43>/UpperRelop'
   *  Switch: '<S43>/Switch'
   */
  if (rtb_Add14 > 100.0) {
    rtb_Add14 = 100.0;
  } else if (rtb_Add14 < -100.0) {
    /* Switch: '<S43>/Switch' incorporates:
     *  Constant: '<S34>/Constant'
     */
    rtb_Add14 = -100.0;
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
  rtb_Add14 -= localDW->DelayInput2_DSTATE_f;

  /* Switch: '<S44>/Switch2' incorporates:
   *  RelationalOperator: '<S44>/LowerRelop1'
   */
  if (!(rtb_Add14 > 1.0)) {
    /* Switch: '<S44>/Switch' incorporates:
     *  RelationalOperator: '<S44>/UpperRelop'
     */
    if (rtb_Add14 < -1.0) {
      rtb_Switch2_b = -1.0;
    } else {
      rtb_Switch2_b = rtb_Add14;
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
  localDW->DelayInput2_DSTATE_f += rtb_Switch2_b;

  /* Saturate: '<S34>/Saturation' incorporates:
   *  UnitDelay: '<S42>/Delay Input2'
   *
   * Block description for '<S42>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (localDW->DelayInput2_DSTATE_f > 0.5) {
    localDW->UnitDelay_DSTATE_p = 0.5;
  } else if (localDW->DelayInput2_DSTATE_f < -0.5) {
    localDW->UnitDelay_DSTATE_p = -0.5;
  } else {
    localDW->UnitDelay_DSTATE_p = localDW->DelayInput2_DSTATE_f;
  }

  /* End of Saturate: '<S34>/Saturation' */

  /* Gain: '<S35>/Gain' incorporates:
   *  UnitDelay: '<S35>/Unit Delay'
   */
  rtb_Gain_mj = 0.3 * localDW->UnitDelay_DSTATE_m;

  /* Product: '<S45>/delta rise limit' incorporates:
   *  Constant: '<S35>/Constant2'
   *  SampleTimeMath: '<S45>/sample time'
   *
   * About '<S45>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Switch2_b = 1.0;

  /* UnitDelay: '<S6>/Unit Delay7' */
  rtb_Add14 = localDW->UnitDelay7_DSTATE;

  /* Product: '<S6>/Divide3' incorporates:
   *  Sum: '<S6>/Add7'
   *  UnitDelay: '<S12>/Unit Delay'
   *  UnitDelay: '<S6>/Unit Delay3'
   */
  localDW->UnitDelay_DSTATE = (rtb_Divide1 - localDW->UnitDelay3_DSTATE_o) /
    0.0099999997764825821;

  /* Sum: '<S6>/Add3' incorporates:
   *  UnitDelay: '<S12>/Unit Delay'
   *  UnitDelay: '<S6>/Unit Delay7'
   */
  localDW->UnitDelay7_DSTATE = localDW->UnitDelay_DSTATE - *rtu_IMU_Ax_mpss;

  /* Product: '<S6>/Divide7' incorporates:
   *  Sum: '<S6>/Add14'
   *  UnitDelay: '<S6>/Unit Delay7'
   */
  rtb_Add14 = (localDW->UnitDelay7_DSTATE - rtb_Add14) / 0.0099999997764825821;

  /* Switch: '<S46>/Switch2' incorporates:
   *  Constant: '<S35>/Constant1'
   *  RelationalOperator: '<S46>/LowerRelop1'
   *  RelationalOperator: '<S46>/UpperRelop'
   *  Switch: '<S46>/Switch'
   */
  if (rtb_Add14 > 100.0) {
    rtb_Add14 = 100.0;
  } else if (rtb_Add14 < -100.0) {
    /* Switch: '<S46>/Switch' incorporates:
     *  Constant: '<S35>/Constant'
     */
    rtb_Add14 = -100.0;
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
  rtb_Add14 -= localDW->DelayInput2_DSTATE_o;

  /* Switch: '<S47>/Switch2' incorporates:
   *  RelationalOperator: '<S47>/LowerRelop1'
   */
  if (!(rtb_Add14 > 1.0)) {
    /* Switch: '<S47>/Switch' incorporates:
     *  RelationalOperator: '<S47>/UpperRelop'
     */
    if (rtb_Add14 < -1.0) {
      rtb_Switch2_b = -1.0;
    } else {
      rtb_Switch2_b = rtb_Add14;
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
  localDW->DelayInput2_DSTATE_o += rtb_Switch2_b;

  /* Saturate: '<S35>/Saturation' incorporates:
   *  UnitDelay: '<S45>/Delay Input2'
   *
   * Block description for '<S45>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (localDW->DelayInput2_DSTATE_o > 0.5) {
    localDW->UnitDelay_DSTATE_m = 0.5;
  } else if (localDW->DelayInput2_DSTATE_o < -0.5) {
    localDW->UnitDelay_DSTATE_m = -0.5;
  } else {
    localDW->UnitDelay_DSTATE_m = localDW->DelayInput2_DSTATE_o;
  }

  /* End of Saturate: '<S35>/Saturation' */

  /* Logic: '<S2>/Logical Operator1' incorporates:
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
   *  Gain: '<S32>/Gain'
   *  Gain: '<S32>/Gain1'
   *  Gain: '<S33>/Gain1'
   *  Gain: '<S34>/Gain1'
   *  Gain: '<S35>/Gain1'
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
   *  Sum: '<S32>/Add'
   *  Sum: '<S33>/Add'
   *  Sum: '<S34>/Add'
   *  Sum: '<S35>/Add'
   *  Sum: '<S7>/Add'
   *  Sum: '<S7>/Add1'
   *  Sum: '<S7>/Add2'
   *  Sum: '<S7>/Add3'
   *  UnitDelay: '<Root>/Unit Delay'
   *  UnitDelay: '<S32>/Unit Delay'
   *  UnitDelay: '<S33>/Unit Delay'
   *  UnitDelay: '<S34>/Unit Delay'
   *  UnitDelay: '<S35>/Unit Delay'
   */
  rtb_LogicalOperator1_l_idx_0 = ((fabs(rtb_VxFL_mps -
    localDW->UnitDelay_DSTATE_e) <= 0.5) && ((rtb_Switch2_ct <= 0.5) || (fabs
    (0.3 * localDW->UnitDelay_DSTATE_dh + 0.7 * rtb_Add7) <= 0.5)));
  rtb_LogicalOperator1_l_idx_1 = ((fabs(rtb_Gain - localDW->UnitDelay_DSTATE_e) <=
    0.5) && ((rtb_Switch2_hz <= 0.5) || (fabs(0.7 * localDW->UnitDelay_DSTATE_l
    + rtb_Gain_e) <= 0.5)));
  rtb_LogicalOperator1_l_idx_2 = ((fabs(rtb_Gain_g - localDW->UnitDelay_DSTATE_e)
    <= 0.5) && ((rtb_Switch2_b2 <= 0.5) || (fabs(0.7 *
    localDW->UnitDelay_DSTATE_p + rtb_Gain_k) <= 0.5)));
  rtb_LogicalOperator1_l_idx_3 = ((fabs(rtb_Divide1 -
    localDW->UnitDelay_DSTATE_e) <= 0.5) && ((fabs(rtb_Switch2_j3) <= 0.5) ||
    (fabs(0.7 * localDW->UnitDelay_DSTATE_m + rtb_Gain_mj) <= 0.5)));

  /* Sum: '<S3>/Add1' incorporates:
   *  Delay: '<S3>/Delay'
   */
  rtb_Add3_c = *rtu_TrqReq_Nm - localDW->Delay_DSTATE[0];

  /* Logic: '<S3>/Logical Operator2' incorporates:
   *  Constant: '<S52>/Constant'
   *  Logic: '<S3>/Logical Operator'
   *  Logic: '<S3>/Logical Operator1'
   *  RelationalOperator: '<S52>/Compare'
   */
  rtb_LogicalOperator2 = (((!rtb_LogicalOperator1_l_idx_0) &&
    (!rtb_LogicalOperator1_l_idx_1) && (!rtb_LogicalOperator1_l_idx_2) &&
    (!rtb_LogicalOperator1_l_idx_3)) || (rtb_Add3_c > 20.0F));

  /* Switch: '<S3>/Switch1' */
  if (rtb_LogicalOperator2) {
    /* Product: '<S3>/Product' incorporates:
     *  Constant: '<S3>/steptime1'
     */
    rtb_Add3_c = *rtu_IMU_Ax_mpss * 0.01F;

    /* Saturate: '<S3>/Saturation1' incorporates:
     *  Sum: '<S3>/Add'
     *  UnitDelay: '<S3>/Unit Delay'
     */
    if (localDW->UnitDelay_DSTATE_pp > 50.0F) {
      localDW->UnitDelay_DSTATE_pp = 50.0F;
    } else if (localDW->UnitDelay_DSTATE_pp < -10.0F) {
      localDW->UnitDelay_DSTATE_pp = -10.0F;
    }

    localDW->UnitDelay_DSTATE_pp += rtb_Add3_c;
  } else {
    /* Saturate: '<S3>/Saturation1' incorporates:
     *  Constant: '<S3>/Constant'
     *  UnitDelay: '<S3>/Unit Delay'
     */
    localDW->UnitDelay_DSTATE_pp = 0.0F;
  }

  /* End of Switch: '<S3>/Switch1' */

  /* Switch: '<S3>/Switch3' incorporates:
   *  Logic: '<S3>/Logical Operator3'
   *  UnitDelay: '<S3>/Unit Delay3'
   */
  if (!localDW->UnitDelay3_DSTATE_j) {
    /* Switch: '<S3>/Switch4' incorporates:
     *  Constant: '<S3>/steptime3'
     *  UnitDelay: '<Root>/Unit Delay1'
     *  UnitDelay: '<S3>/Unit Delay4'
     */
    if (rtb_LogicalOperator2) {
      localDW->UnitDelay4_DSTATE_d = localDW->UnitDelay1_DSTATE_i;
    } else {
      localDW->UnitDelay4_DSTATE_d = 0.0F;
    }

    /* End of Switch: '<S3>/Switch4' */
  }

  /* End of Switch: '<S3>/Switch3' */

  /* Saturate: '<S3>/Saturation' incorporates:
   *  UnitDelay: '<S3>/Unit Delay'
   */
  if (localDW->UnitDelay_DSTATE_pp > 50.0F) {
    rtb_Product1 = 50.0F;
  } else if (localDW->UnitDelay_DSTATE_pp < -10.0F) {
    rtb_Product1 = -10.0F;
  } else {
    rtb_Product1 = localDW->UnitDelay_DSTATE_pp;
  }

  /* Sum: '<S3>/Add3' incorporates:
   *  Saturate: '<S3>/Saturation'
   *  UnitDelay: '<S3>/Unit Delay4'
   */
  rtb_Add3_c = rtb_Product1 + localDW->UnitDelay4_DSTATE_d;

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
  if (fmax(fmax(fmax(rtb_VxFL_mps, rtb_Gain), rtb_Gain_g), rtb_Divide1) -
      rtb_Add3_c <= 0.0) {
    rtb_Product1 = localDW->UnitDelay2_DSTATE_h + 0.01F;
  } else {
    rtb_Product1 = 0.0F;
  }

  /* MinMax: '<S3>/Min' incorporates:
   *  Constant: '<S3>/ResetDelay'
   *  Switch: '<S3>/Switch6'
   *  UnitDelay: '<S3>/Unit Delay2'
   */
  localDW->UnitDelay2_DSTATE_h = fminf(rtb_Product1, 0.1F);

  /* Switch: '<S4>/Switch1' incorporates:
   *  Constant: '<S4>/Constant'
   */
  if (rtb_LogicalOperator1_l_idx_0) {
    rtb_Add14 = rtb_VxFL_mps;
  } else {
    rtb_Add14 = (rtNaN);
  }

  /* End of Switch: '<S4>/Switch1' */

  /* Switch: '<S4>/Switch2' incorporates:
   *  Constant: '<S4>/Constant1'
   */
  if (rtb_LogicalOperator1_l_idx_1) {
    rtb_Switch2_hz = rtb_Gain;
  } else {
    rtb_Switch2_hz = (rtNaN);
  }

  /* End of Switch: '<S4>/Switch2' */

  /* Switch: '<S4>/Switch3' incorporates:
   *  Constant: '<S4>/Constant2'
   */
  if (rtb_LogicalOperator1_l_idx_2) {
    rtb_Switch2_b2 = rtb_Gain_g;
  } else {
    rtb_Switch2_b2 = (rtNaN);
  }

  /* End of Switch: '<S4>/Switch3' */

  /* Switch: '<S4>/Switch4' incorporates:
   *  Constant: '<S4>/Constant3'
   */
  if (rtb_LogicalOperator1_l_idx_3) {
    rtb_Switch2_b = rtb_Divide1;
  } else {
    rtb_Switch2_b = (rtNaN);
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
  rtb_Switch2_ct = (((rtb_Add14 + rtb_Switch2_hz) + rtb_Switch2_b2) +
                    rtb_Switch2_b) / ((((real_T)rtb_LogicalOperator1_l_idx_0 +
    (real_T)rtb_LogicalOperator1_l_idx_1) + (real_T)rtb_LogicalOperator1_l_idx_2)
    + (real_T)rtb_LogicalOperator1_l_idx_3);

  /* UnitDelay: '<S57>/Unit Delay' */
  rtb_UnitDelay_a = localDW->UnitDelay_DSTATE_pz;

  /* Product: '<S61>/delta rise limit' incorporates:
   *  Constant: '<S57>/Constant2'
   *  SampleTimeMath: '<S61>/sample time'
   *
   * About '<S61>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Product1 = 1.0F;

  /* DataTypeConversion: '<S4>/Cast To Double4' incorporates:
   *  Sum: '<S4>/Add2'
   *  UnitDelay: '<S4>/Unit Delay'
   */
  rtb_UkYk1_de = (real32_T)(rtb_Switch2_ct - localDW->UnitDelay_DSTATE_hv);

  /* Switch: '<S62>/Switch2' incorporates:
   *  Constant: '<S57>/Constant'
   *  Constant: '<S57>/Constant1'
   *  DataTypeConversion: '<S4>/Cast To Double4'
   *  RelationalOperator: '<S62>/LowerRelop1'
   *  RelationalOperator: '<S62>/UpperRelop'
   *  Switch: '<S62>/Switch'
   */
  if (rtb_UkYk1_de > 100.0F) {
    rtb_UkYk1_de = 100.0F;
  } else if (rtb_UkYk1_de < -100.0F) {
    /* Switch: '<S62>/Switch' incorporates:
     *  Constant: '<S57>/Constant'
     */
    rtb_UkYk1_de = -100.0F;
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
  rtb_UkYk1_de -= localDW->DelayInput2_DSTATE_b;

  /* Switch: '<S63>/Switch2' incorporates:
   *  RelationalOperator: '<S63>/LowerRelop1'
   */
  if (!(rtb_UkYk1_de > 1.0F)) {
    /* Switch: '<S63>/Switch' incorporates:
     *  RelationalOperator: '<S63>/UpperRelop'
     */
    if (rtb_UkYk1_de < -1.0F) {
      rtb_Product1 = -1.0F;
    } else {
      rtb_Product1 = rtb_UkYk1_de;
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
  localDW->DelayInput2_DSTATE_b += rtb_Product1;

  /* Saturate: '<S57>/Saturation' incorporates:
   *  UnitDelay: '<S61>/Delay Input2'
   *
   * Block description for '<S61>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (localDW->DelayInput2_DSTATE_b > 0.5F) {
    localDW->UnitDelay_DSTATE_pz = 0.5F;
  } else if (localDW->DelayInput2_DSTATE_b < -0.5F) {
    localDW->UnitDelay_DSTATE_pz = -0.5F;
  } else {
    localDW->UnitDelay_DSTATE_pz = localDW->DelayInput2_DSTATE_b;
  }

  /* End of Saturate: '<S57>/Saturation' */

  /* Switch: '<S4>/Switch' incorporates:
   *  Constant: '<S3>/ResetDelay'
   *  Logic: '<S4>/Logical Operator'
   *  Logic: '<S4>/Logical Operator1'
   *  RelationalOperator: '<S3>/Relational Operator9'
   *  UnitDelay: '<S3>/Unit Delay2'
   */
  if (rtb_LogicalOperator1_l_idx_0 || rtb_LogicalOperator1_l_idx_1 ||
      rtb_LogicalOperator1_l_idx_2 || rtb_LogicalOperator1_l_idx_3 ||
      (localDW->UnitDelay2_DSTATE_h >= 0.1F)) {
    /* Sum: '<S57>/Add' incorporates:
     *  Gain: '<S57>/Gain'
     *  Gain: '<S57>/Gain1'
     *  UnitDelay: '<S57>/Unit Delay'
     */
    rtb_Add3_c = 0.3F * rtb_UnitDelay_a + 0.7F * localDW->UnitDelay_DSTATE_pz;

    /* Switch: '<S4>/Switch6' incorporates:
     *  Constant: '<S55>/Constant'
     *  Logic: '<S4>/Logical Operator2'
     *  MinMax: '<S4>/Min'
     *  RelationalOperator: '<S55>/Compare'
     */
    if (!(rtb_Add3_c > 0.5F)) {
      /* Switch: '<S4>/Switch5' incorporates:
       *  Constant: '<S54>/Constant'
       *  MinMax: '<S4>/Min1'
       *  RelationalOperator: '<S54>/Compare'
       */
      if (rtb_Add3_c < -0.5F) {
        rtb_Add3_c = (real32_T)fmax(fmax(fmax(rtb_Add14, rtb_Switch2_hz),
          rtb_Switch2_b2), rtb_Switch2_b);
      } else {
        rtb_Add3_c = (real32_T)rtb_Switch2_ct;
      }

      /* End of Switch: '<S4>/Switch5' */
    } else {
      rtb_Add3_c = (real32_T)fmin(fmin(fmin(rtb_Add14, rtb_Switch2_hz),
        rtb_Switch2_b2), rtb_Switch2_b);
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
  if (rtb_Add3_c > 100.0F) {
    rtb_Add3_c = 100.0F;
  } else if (rtb_Add3_c < -100.0F) {
    /* Switch: '<S59>/Switch' incorporates:
     *  Constant: '<S56>/Constant'
     */
    rtb_Add3_c = -100.0F;
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
  rtb_Add3_c -= localDW->DelayInput2_DSTATE_c4;

  /* Product: '<S58>/delta rise limit' incorporates:
   *  Constant: '<S56>/Constant2'
   *  SampleTimeMath: '<S58>/sample time'
   *
   * About '<S58>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Product1 = 1.0F;

  /* Switch: '<S60>/Switch2' incorporates:
   *  RelationalOperator: '<S60>/LowerRelop1'
   */
  if (!(rtb_Add3_c > 1.0F)) {
    /* Switch: '<S60>/Switch' incorporates:
     *  RelationalOperator: '<S60>/UpperRelop'
     */
    if (rtb_Add3_c < -1.0F) {
      rtb_Product1 = -1.0F;
    } else {
      rtb_Product1 = rtb_Add3_c;
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
  localDW->DelayInput2_DSTATE_c4 += rtb_Product1;

  /* Saturate: '<S56>/Saturation' incorporates:
   *  UnitDelay: '<S58>/Delay Input2'
   *
   * Block description for '<S58>/Delay Input2':
   *
   *  Store in Global RAM
   */
  if (localDW->DelayInput2_DSTATE_c4 > 0.5F) {
    rtb_Add3_c = 0.5F;
  } else if (localDW->DelayInput2_DSTATE_c4 < -0.5F) {
    rtb_Add3_c = -0.5F;
  } else {
    rtb_Add3_c = localDW->DelayInput2_DSTATE_c4;
  }

  /* End of Saturate: '<S56>/Saturation' */

  /* Sum: '<S56>/Add' incorporates:
   *  Gain: '<S56>/Gain'
   *  Gain: '<S56>/Gain1'
   *  UnitDelay: '<S56>/Unit Delay'
   */
  *rty_VehVxEst_mps = 0.3F * localDW->UnitDelay_DSTATE_o + 0.7F * rtb_Add3_c;

  /* Update for UnitDelay: '<S12>/Unit Delay' */
  localDW->UnitDelay_DSTATE = rtb_Divide;

  /* Update for UnitDelay: '<S5>/Unit Delay' */
  localDW->UnitDelay_DSTATE_d = rtb_VxFL_mps;

  /* Update for UnitDelay: '<S5>/Unit Delay1' */
  localDW->UnitDelay1_DSTATE = rtb_Gain;

  /* Update for UnitDelay: '<S14>/Unit Delay' */
  localDW->UnitDelay_DSTATE_h = rtb_Saturation_gy;

  /* Update for UnitDelay: '<S5>/Unit Delay2' */
  localDW->UnitDelay2_DSTATE = rtb_Gain_g;

  /* Update for UnitDelay: '<S5>/Unit Delay3' */
  localDW->UnitDelay3_DSTATE = rtb_Divide1;

  /* Update for UnitDelay: '<S32>/Unit Delay' */
  localDW->UnitDelay_DSTATE_dh = rtb_Add7;

  /* Update for UnitDelay: '<S6>/Unit Delay' */
  localDW->UnitDelay_DSTATE_kf = rtb_VxFL_mps;

  /* Update for UnitDelay: '<S6>/Unit Delay1' */
  localDW->UnitDelay1_DSTATE_k = rtb_Gain;

  /* Update for UnitDelay: '<S6>/Unit Delay2' */
  localDW->UnitDelay2_DSTATE_p = rtb_Gain_g;

  /* Update for UnitDelay: '<S6>/Unit Delay3' */
  localDW->UnitDelay3_DSTATE_o = rtb_Divide1;

  /* Update for UnitDelay: '<Root>/Unit Delay' */
  localDW->UnitDelay_DSTATE_e = *rty_VehVxEst_mps;

  /* Update for Delay: '<S3>/Delay' */
  localDW->Delay_DSTATE[0] = localDW->Delay_DSTATE[1];
  localDW->Delay_DSTATE[1] = localDW->Delay_DSTATE[2];
  localDW->Delay_DSTATE[2] = localDW->Delay_DSTATE[3];
  localDW->Delay_DSTATE[3] = localDW->Delay_DSTATE[4];
  localDW->Delay_DSTATE[4] = *rtu_TrqReq_Nm;

  /* Update for UnitDelay: '<Root>/Unit Delay1' */
  localDW->UnitDelay1_DSTATE_i = *rty_VehVxEst_mps;

  /* Update for UnitDelay: '<S3>/Unit Delay3' */
  localDW->UnitDelay3_DSTATE_j = rtb_LogicalOperator2;

  /* Update for UnitDelay: '<S4>/Unit Delay' */
  localDW->UnitDelay_DSTATE_hv = rtb_Switch2_ct;

  /* Update for UnitDelay: '<S56>/Unit Delay' */
  localDW->UnitDelay_DSTATE_o = rtb_Add3_c;
}

/* Model initialize function */
void VxSpdEsti0713_initialize(const char_T **rt_errorStatus,
  RT_MODEL_VxSpdEsti0713_T *const VxSpdEsti0713_M)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatusPointer(VxSpdEsti0713_M, rt_errorStatus);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
