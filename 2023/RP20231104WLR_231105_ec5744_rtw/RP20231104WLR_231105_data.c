/*
 * Code generated for Simulink model RP20231104WLR_231105.
 *
 * FILE    : RP20231104WLR_231105_data.c
 *
 * VERSION : 1.1
 *
 * DATE    : Sat Sep 21 10:42:01 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#include "RP20231104WLR_231105.h"
#include "RP20231104WLR_231105_private.h"

/* Constant parameters (default storage) */
const ConstP_RP20231104WLR_231105_T RP20231104WLR_231105_ConstP = {
  /* Expression: [0.3,0.3,2.5,2.5]
   * Referenced by: '<S14>/VehSpd_SlipTarget_mps'
   */
  { 0.3, 0.3, 2.5, 2.5 },

  /* Pooled Parameter (Expression: [0,3,25,30])
   * Referenced by:
   *   '<S14>/VehSpd_SlipTarget_mps'
   *   '<S14>/VehicleStableTarget_mps'
   *   '<S14>/VehicleStableTarget_mps1'
   */
  { 0.0, 3.0, 25.0, 30.0 },

  /* Pooled Parameter (Expression: [0.4,0.4,1.2,1.2])
   * Referenced by:
   *   '<S14>/VehicleStableTarget_mps'
   *   '<S14>/VehicleStableTarget_mps1'
   */
  { 0.4, 0.4, 1.2, 1.2 },

  /* Expression: single([1000,0]);
   * Referenced by: '<S8>/BrakeCompensateCoef'
   */
  { 1000.0F, 0.0F },

  /* Expression: single([550,2000]);
   * Referenced by: '<S8>/BrakeCompensateCoef'
   */
  { 550.0F, 2000.0F },

  /* Expression: single(reshape([0,20,35,50,79,100,120,125,130,130,170,0,20,35,50,79,100,120,125,160,160,170,0,20,35,50,79,100,120,125,160,160,180,0,20,35,50,79,105,120,130,160,160,180,0,20,35,50,85,105,125,135,160,160,180,0,20,35,50,85,105,125,140,170,170,210,0,20,30,50,85,110,125,145,160,160,200,0,18,30,50,90,110,135,155,160,160,200,0,18,25,50,95,110,135,155,160,160,200,0,18,25,40,100,110,135,155,160,160,200,0,17,25,40,110,115,135,155,160,160,200,0,17,25,40,110,115,135,155,160,160,200,0,16,20,40,110,125,135,155,160,160,190,0,16,20,40,100,125,130,155,160,160,180,0,15,20,40,90,120,130,155,160,160,180,0,15,20,40,90,110,120,135,142,142,152,0,15,20,40,80,90,100,110,120,120,133,0,14,20,40,72,80,90,100,105,105,118,0,14,20,38,70,76,82,88,94,95,106,0,14,18,35,65,70,74,82,88,91,100],11,20));
   * Referenced by: '<S8>/2-D Lookup Table'
   */
  { 0.0F, 20.0F, 35.0F, 50.0F, 79.0F, 100.0F, 120.0F, 125.0F, 130.0F, 130.0F,
    170.0F, 0.0F, 20.0F, 35.0F, 50.0F, 79.0F, 100.0F, 120.0F, 125.0F, 160.0F,
    160.0F, 170.0F, 0.0F, 20.0F, 35.0F, 50.0F, 79.0F, 100.0F, 120.0F, 125.0F,
    160.0F, 160.0F, 180.0F, 0.0F, 20.0F, 35.0F, 50.0F, 79.0F, 105.0F, 120.0F,
    130.0F, 160.0F, 160.0F, 180.0F, 0.0F, 20.0F, 35.0F, 50.0F, 85.0F, 105.0F,
    125.0F, 135.0F, 160.0F, 160.0F, 180.0F, 0.0F, 20.0F, 35.0F, 50.0F, 85.0F,
    105.0F, 125.0F, 140.0F, 170.0F, 170.0F, 210.0F, 0.0F, 20.0F, 30.0F, 50.0F,
    85.0F, 110.0F, 125.0F, 145.0F, 160.0F, 160.0F, 200.0F, 0.0F, 18.0F, 30.0F,
    50.0F, 90.0F, 110.0F, 135.0F, 155.0F, 160.0F, 160.0F, 200.0F, 0.0F, 18.0F,
    25.0F, 50.0F, 95.0F, 110.0F, 135.0F, 155.0F, 160.0F, 160.0F, 200.0F, 0.0F,
    18.0F, 25.0F, 40.0F, 100.0F, 110.0F, 135.0F, 155.0F, 160.0F, 160.0F, 200.0F,
    0.0F, 17.0F, 25.0F, 40.0F, 110.0F, 115.0F, 135.0F, 155.0F, 160.0F, 160.0F,
    200.0F, 0.0F, 17.0F, 25.0F, 40.0F, 110.0F, 115.0F, 135.0F, 155.0F, 160.0F,
    160.0F, 200.0F, 0.0F, 16.0F, 20.0F, 40.0F, 110.0F, 125.0F, 135.0F, 155.0F,
    160.0F, 160.0F, 190.0F, 0.0F, 16.0F, 20.0F, 40.0F, 100.0F, 125.0F, 130.0F,
    155.0F, 160.0F, 160.0F, 180.0F, 0.0F, 15.0F, 20.0F, 40.0F, 90.0F, 120.0F,
    130.0F, 155.0F, 160.0F, 160.0F, 180.0F, 0.0F, 15.0F, 20.0F, 40.0F, 90.0F,
    110.0F, 120.0F, 135.0F, 142.0F, 142.0F, 152.0F, 0.0F, 15.0F, 20.0F, 40.0F,
    80.0F, 90.0F, 100.0F, 110.0F, 120.0F, 120.0F, 133.0F, 0.0F, 14.0F, 20.0F,
    40.0F, 72.0F, 80.0F, 90.0F, 100.0F, 105.0F, 105.0F, 118.0F, 0.0F, 14.0F,
    20.0F, 38.0F, 70.0F, 76.0F, 82.0F, 88.0F, 94.0F, 95.0F, 106.0F, 0.0F, 14.0F,
    18.0F, 35.0F, 65.0F, 70.0F, 74.0F, 82.0F, 88.0F, 91.0F, 100.0F },

  /* Expression: single([0,10,20,30,40,50,60,70,80,90,100]);
   * Referenced by: '<S8>/2-D Lookup Table'
   */
  { 0.0F, 10.0F, 20.0F, 30.0F, 40.0F, 50.0F, 60.0F, 70.0F, 80.0F, 90.0F, 100.0F
  },

  /* Expression: single([200,400,600,800,1000,1200,1400,1600,1800,2000,2200,2400,2600,2800,3000,3500,4000,4500,5000,5200]);
   * Referenced by: '<S8>/2-D Lookup Table'
   */
  { 200.0F, 400.0F, 600.0F, 800.0F, 1000.0F, 1200.0F, 1400.0F, 1600.0F, 1800.0F,
    2000.0F, 2200.0F, 2400.0F, 2600.0F, 2800.0F, 3000.0F, 3500.0F, 4000.0F,
    4500.0F, 5000.0F, 5200.0F },

  /* Computed Parameter: MCU__spd_tableData
   * Referenced by: '<S7>/MCU__spd'
   */
  { 0.0F, 0.0F, 600.0F, 600.0F, 600.0F, 600.0F, 800.0F, 800.0F, 800.0F, 1000.0F,
    1000.0F, 1000.0F, 1000.0F, 1000.0F },

  /* Computed Parameter: MCU__spd_bp01Data
   * Referenced by: '<S7>/MCU__spd'
   */
  { 10.0F, 15.0F, 20.0F, 25.0F, 30.0F, 35.0F, 40.0F, 45.0F, 50.0F, 55.0F, 60.0F,
    65.0F, 70.0F, 75.0F },

  /* Computed Parameter: moter__spd_tableData
   * Referenced by: '<S7>/moter__spd'
   */
  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1400.0F, 1700.0F, 2000.0F, 2000.0F,
    2000.0F, 2000.0F },

  /* Computed Parameter: moter__spd_bp01Data
   * Referenced by: '<S7>/moter__spd'
   */
  { 0.0F, 10.0F, 20.0F, 30.0F, 40.0F, 50.0F, 60.0F, 70.0F, 80.0F, 90.0F, 100.0F,
    110.0F, 120.0F },

  /* Pooled Parameter (Expression: single([0,100]);)
   * Referenced by:
   *   '<S23>/1-D Lookup Table'
   *   '<S23>/1-D Lookup Table3'
   */
  { 0.0F, 100.0F },

  /* Expression: single([1330,1590]);
   * Referenced by: '<S23>/1-D Lookup Table'
   */
  { 1330.0F, 1590.0F },

  /* Expression: single([2324,2600]);
   * Referenced by: '<S23>/1-D Lookup Table3'
   */
  { 2324.0F, 2600.0F },

  /* Pooled Parameter (Expression: single([0,10000]);)
   * Referenced by:
   *   '<S23>/1-D Lookup Table1'
   *   '<S23>/1-D Lookup Table2'
   */
  { 0.0F, 10000.0F },

  /* Expression: single([530,4500]);
   * Referenced by: '<S23>/1-D Lookup Table2'
   */
  { 530.0F, 4500.0F },

  /* Expression: single([500,4500]);
   * Referenced by: '<S23>/1-D Lookup Table1'
   */
  { 500.0F, 4500.0F },

  /* Computed Parameter: uDLookupTable_maxIndex
   * Referenced by: '<S8>/2-D Lookup Table'
   */
  { 10U, 19U }
};

/* File trailer for ECUCoder generated file RP20231104WLR_231105_data.c.
 *
 * [EOF]
 */
