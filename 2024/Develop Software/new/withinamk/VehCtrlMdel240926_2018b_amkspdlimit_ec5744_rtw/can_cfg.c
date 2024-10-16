/*
 * Code generated for Simulink model VehCtrlMdel240926_2018b_amkspdlimit.
 *
 * FILE    : can_cfg.c
 *
 * VERSION : 1.237
 *
 * DATE    : Sun Oct 13 20:51:54 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#include "can.h"

CAN_Initialization CANINIT[3]= {
  {
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    RX_MB_CODE_EMPTY<<24|3<<21,
    TX_MB_CODE_INACITVE<<24,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    RX_MB_CODE_EMPTY<<24|3<<21,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24
  },

  {
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    RX_MB_CODE_EMPTY<<24|0<<21,
    TX_MB_CODE_INACITVE<<24,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24
  },

  {
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    RX_MB_CODE_EMPTY<<24|0<<21,
    TX_MB_CODE_INACITVE<<24,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    RX_MB_CODE_EMPTY<<24|0<<21,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24,
    TX_MB_CODE_INACITVE<<24
  }
};

CAN_Int CANIF = {
  0x00000000,
  0x00000000,
  0x00000000,
};

/* File trailer for ECUCoder generated file can_cfg.c.
 *
 * [EOF]
 */
