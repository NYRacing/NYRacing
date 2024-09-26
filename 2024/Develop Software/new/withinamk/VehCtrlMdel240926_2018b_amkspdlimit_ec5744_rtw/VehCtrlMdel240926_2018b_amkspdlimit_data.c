/*
 * Code generated for Simulink model VehCtrlMdel240926_2018b_amkspdlimit.
 *
 * FILE    : VehCtrlMdel240926_2018b_amkspdlimit_data.c
 *
 * VERSION : 1.174
 *
 * DATE    : Thu Sep 26 14:50:31 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#include "VehCtrlMdel240926_2018b_amkspdlimit.h"
#include "VehCtrlMdel240926_2018b_amkspdlimit_private.h"

/* Constant parameters (default storage) */
const ConstP_VehCtrlMdel240926_2018_T VehCtrlMdel240926_2018b__ConstP = {
  /* Expression: [229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.85;229.46;229.16;228.86;228.86;228.86;228.86;228.86;227.86;227.86;226.87;225.86;224.86;220.86;218.87;215.87]
   * Referenced by: '<S10>/228'
   */
  { 229.85, 229.85, 229.85, 229.85, 229.85, 229.85, 229.85, 229.85, 229.85,
    229.85, 229.85, 229.85, 229.46, 229.16, 228.86, 228.86, 228.86, 228.86,
    228.86, 227.86, 227.86, 226.87, 225.86, 224.86, 220.86, 218.87, 215.87 },

  /* Expression: [200;400;600;800;1000;1200;1400;1600;1800;2000;2200;2400;2600;2800;3000;3200;3400;3600;3800;4000;4200;4400;4600;4800;5000;5200;5400]
   * Referenced by: '<S10>/228'
   */
  { 200.0, 400.0, 600.0, 800.0, 1000.0, 1200.0, 1400.0, 1600.0, 1800.0, 2000.0,
    2200.0, 2400.0, 2600.0, 2800.0, 3000.0, 3200.0, 3400.0, 3600.0, 3800.0,
    4000.0, 4200.0, 4400.0, 4600.0, 4800.0, 5000.0, 5200.0, 5400.0 },

  /* Pooled Parameter (Expression: [21;21;21;21;21;21;21;21;21;21;21;21;21;21;20;17.5;15;12.5;8;0])
   * Referenced by:
   *   '<S10>/AMK'
   *   '<S10>/AMK1'
   */
  { 21.0, 21.0, 21.0, 21.0, 21.0, 21.0, 21.0, 21.0, 21.0, 21.0, 21.0, 21.0, 21.0,
    21.0, 20.0, 17.5, 15.0, 12.5, 8.0, 0.0 },

  /* Pooled Parameter (Expression: [0;1000;2000;3000;4000;5000;6000;7000;8000;9000;10000;11000;12000;13000;14000;15000;16000;17000;18000;19000])
   * Referenced by:
   *   '<S10>/AMK'
   *   '<S10>/AMK1'
   */
  { 0.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0, 8000.0, 9000.0,
    10000.0, 11000.0, 12000.0, 13000.0, 14000.0, 15000.0, 16000.0, 17000.0,
    18000.0, 19000.0 },

  /* Pooled Parameter (Expression: [0,0,0.2,0.4,0.6,0.8,1,1])
   * Referenced by:
   *   '<S8>/2-D Lookup Table1'
   *   '<S8>/2-D Lookup Table3'
   */
  { 0.0, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.0 },

  /* Pooled Parameter (Expression: [35,40,41,42,43,44,45,50])
   * Referenced by:
   *   '<S8>/2-D Lookup Table1'
   *   '<S8>/2-D Lookup Table3'
   */
  { 35.0, 40.0, 41.0, 42.0, 43.0, 44.0, 45.0, 50.0 },

  /* Expression: [0;0;20;40;70;100;100]
   * Referenced by: '<S8>/2-D Lookup Table2'
   */
  { 0.0, 0.0, 20.0, 40.0, 70.0, 100.0, 100.0 },

  /* Expression: [0;25;30;35;37;40;50]
   * Referenced by: '<S8>/2-D Lookup Table2'
   */
  { 0.0, 25.0, 30.0, 35.0, 37.0, 40.0, 50.0 },

  /* Computed Parameter: uDLookupTable_tableData
   * Referenced by: '<S203>/1-D Lookup Table'
   */
  { -26.072927474975586, -23.622480392456055, -20.834568023681641,
    -18.0024471282959, -15.197903633117676, -12.620231628417969,
    -10.131048202514648, -8.13967227935791, -6.2976670265197754,
    -4.6050119400024414, -3.0893688201904297, -1.4907528162002563,
    -0.074676238000392914, 1.4077800512313843, 2.9621448516845703,
    4.7211766242980957, 6.5521149635314941, 8.737055778503418,
    10.944144248962402, 13.743144035339355, 17.250120162963867,
    20.375423431396484, 24.070535659790039, 27.505584716796875 },

  /* Computed Parameter: uDLookupTable_bp01Data
   * Referenced by: '<S203>/1-D Lookup Table'
   */
  { -119.99299621582031, -109.99700164794922, -100.0, -90.003402709960938,
    -80.006698608398438, -69.993301391601563, -59.996601104736328, -50.0,
    -40.003398895263672, -30.00670051574707, -19.99329948425293,
    -9.9966402053833, 0.0, 9.9966402053833, 19.99329948425293, 30.00670051574707,
    40.003398895263672, 50.0, 59.996601104736328, 69.993301391601563,
    80.006698608398438, 90.003402709960938, 100.0, 109.99700164794922 },

  /* Computed Parameter: uDLookupTable1_tableData
   * Referenced by: '<S203>/1-D Lookup Table1'
   */
  { -26.816255569458008, -23.524848937988281, -19.851047515869141,
    -16.282583236694336, -13.406760215759277, -10.314791679382324,
    -7.6550397872924805, -5.355496883392334, -3.4548912048339844,
    -1.5044183731079102, 0.0249350406229496, 1.43791925907135,
    2.8675296306610107, 4.26389741897583, 5.7600002288818359, 7.3558797836303711,
    9.1013040542602539, 11.046239852905273, 12.941280364990234,
    15.357239723205566, 18.244152069091797, 20.937168121337891,
    23.630111694335938, 26.306495666503906 },

  /* Computed Parameter: uDLookupTable1_bp01Data
   * Referenced by: '<S203>/1-D Lookup Table1'
   */
  { -119.99500274658203, -109.99299621582031, -99.9916000366211,
    -90.006698608398438, -80.004997253417969, -70.003402709960938,
    -60.001701354980469, -50.0, -39.998298645019531, -29.996599197387695,
    -19.9950008392334, -9.9932804107666016, 0.00840476993471384,
    9.9932804107666016, 19.9950008392334, 29.996599197387695, 39.998298645019531,
    50.0, 60.001701354980469, 70.003402709960938, 80.004997253417969,
    90.006698608398438, 100.00800323486328, 109.99299621582031 },

  /* Pooled Parameter (Expression: single([20,0]);)
   * Referenced by:
   *   '<S7>/BrakeCompensateCoefFront1'
   *   '<S7>/BrakeCompensateCoefFront2'
   */
  { 20.0F, 0.0F },

  /* Pooled Parameter (Expression: single([550,1700]);)
   * Referenced by:
   *   '<S7>/BrakeCompensateCoefFront1'
   *   '<S7>/BrakeCompensateCoefFront2'
   */
  { 550.0F, 1700.0F },

  /* Expression: single([0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;20 20 20 20 20 20 20 18 18 18 17 17 16 16 15 15 15 14 14 14;35 35 35 35 35 35 30 30 25 25 25 25 20 20 20 20 20 20 20 18;50 50 50 50 50 50 50 50 50 40 40 40 40 40 40 40 40 40 38 35;79 79 79 79 85 85 85 90 95 100 110 110 110 100 90 90 80 72 70 65;100 100 100 105 105 105 110 110 110 110 115 115 125 125 120 110 90 80 76 70;120 120 120 120 125 125 125 135 135 135 135 135 135 130 130 120 100 90 82 74;125 125 125 130 135 140 145 155 155 155 155 155 155 155 155 135 110 100 88 82;130 160 160 160 160 170 160 160 160 160 160 160 160 160 160 142 120 105 94 88;130 160 160 160 160 170 160 160 160 160 160 160 160 160 160 142 120 105 95 91;170 170 180 180 180 210 200 200 200 200 200 200 190 180 180 152 133 118 106 100])
   * Referenced by: '<S7>/2-D Lookup Table1'
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

  /* Expression: single([0 10 20 30 40 50 60 70 80 90 100])
   * Referenced by: '<S7>/2-D Lookup Table1'
   */
  { 0.0F, 10.0F, 20.0F, 30.0F, 40.0F, 50.0F, 60.0F, 70.0F, 80.0F, 90.0F, 100.0F
  },

  /* Expression: single([5.40000009536743 10.8000001907349 16.2000007629395 21.6000003814697 27 32.4000015258789 37.7999992370605 43.2000007629395 48.5999984741211 54 59.4000015258789 64.8000030517578 70.1999969482422 75.5999984741211 81 86.4000015258789 91.8000030517578 97.1999969482422 102.599998474121 108])
   * Referenced by: '<S7>/2-D Lookup Table1'
   */
  { 5.4F, 10.8F, 16.2F, 21.6F, 27.0F, 32.4F, 37.8F, 43.2F, 48.6F, 54.0F, 59.4F,
    64.8F, 70.2F, 75.6F, 81.0F, 86.4F, 91.8F, 97.2F, 102.6F, 108.0F },

  /* Pooled Parameter (Expression: [0.3,0.3,2.5,2.5])
   * Referenced by:
   *   '<S27>/VehSpd_SlipTarget_mps'
   *   '<S28>/VehSpd_SlipTarget_mps'
   *   '<S29>/VehSpd_SlipTarget_mps'
   */
  { 0.3F, 0.3F, 2.5F, 2.5F },

  /* Pooled Parameter (Expression: [0,3,25,30])
   * Referenced by:
   *   '<S27>/VehSpd_SlipTarget_mps'
   *   '<S27>/VehicleStableTarget_mps'
   *   '<S27>/VehicleStableTarget_mps1'
   *   '<S28>/VehSpd_SlipTarget_mps'
   *   '<S28>/VehicleStableTarget_mps'
   *   '<S28>/VehicleStableTarget_mps1'
   *   '<S29>/VehSpd_SlipTarget_mps'
   *   '<S29>/VehicleStableTarget_mps'
   *   '<S29>/VehicleStableTarget_mps1'
   */
  { 0.0F, 3.0F, 25.0F, 30.0F },

  /* Pooled Parameter (Expression: [0.4,0.4,1.2,1.2])
   * Referenced by:
   *   '<S27>/VehicleStableTarget_mps'
   *   '<S27>/VehicleStableTarget_mps1'
   *   '<S28>/VehicleStableTarget_mps'
   *   '<S28>/VehicleStableTarget_mps1'
   *   '<S29>/VehicleStableTarget_mps'
   *   '<S29>/VehicleStableTarget_mps1'
   */
  { 0.4F, 0.4F, 1.2F, 1.2F },

  /* Expression: single([1000,0]);
   * Referenced by: '<S7>/BrakeCompensateCoefRear'
   */
  { 1000.0F, 0.0F },

  /* Expression: single([550,1500]);
   * Referenced by: '<S7>/BrakeCompensateCoefRear'
   */
  { 550.0F, 1500.0F },

  /* Pooled Parameter (Expression: single([500,4500]);)
   * Referenced by: '<S201>/1-D Lookup Table1'
   */
  { 500.0F, 4500.0F },

  /* Expression: single([0,100]);
   * Referenced by: '<S201>/1-D Lookup Table3'
   */
  { 0.0F, 100.0F },

  /* Expression: single([3540,3790])
   * Referenced by: '<S201>/1-D Lookup Table3'
   */
  { 3540.0F, 3790.0F },

  /* Computed Parameter: uDLookupTable1_maxIndex
   * Referenced by: '<S7>/2-D Lookup Table1'
   */
  { 10U, 19U }
};

/* File trailer for ECUCoder generated file VehCtrlMdel240926_2018b_amkspdlimit_data.c.
 *
 * [EOF]
 */
