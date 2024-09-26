/*
 * Code generated for Simulink model VehCtrlMdel240918_2018b.
 *
 * FILE    : spi.c
 *
 * VERSION : 1.169
 *
 * DATE    : Wed Sep 25 22:39:30 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#include "spi.h"
#if (SPIAENABLE||SPIBENABLE)

/**************************************************************************
 * FUNCTION :    ec_spi_delay                                              *
 *                                                                         *
 * DESCRIPTION : A delay function. Wait until get RFDF flag                *
 *                                                                         *
 * PARAMETERS :  -DSPIn: can be DSPI0, or DSPI1                 *
 *               -us: the number of us to be delayed                       *
 *                                                                         *
 * RETURN :      None                                                      *
 ***************************************************************************/
void ec_spi_delay(uint8 DSPIn,uint32 us)
{
  uint32 i,j;
  for (i=0; i<us; i++) {
    for (j=0; j<20; j++) {
      if (1 == DSPI(DSPIn).SR.B.RFDF) {
        i = us+1;
        j = 21;
      }
    }
  }
}

#endif

#if 1

/**************************************************************************
 * FUNCTION :    ec_spi0_init                                              *
 *                                                                         *
 * DESCRIPTION : Initialize DSPI0 module                                   *
 *                                                                         *
 * PARAMETERS :  None                                                      *
 *                                                                         *
 * RETURN:       None                                                      *
 ***************************************************************************/
void ec_spi0_init(void)
{
  DSPI(0).MCR.B.MSTR = 1;              /* SPI Mode, Master or Slave */
  DSPI(0).MCR.B.CONT_SCKE = 0;
  DSPI(0).MCR.B.DCONF = 0;
                         /* Configure DSPI_A in SPI, DSI or CSI configuration */
  DSPI(0).MCR.B.PCSIS = 0b11111111;
  DSPI(0).MCR.B.MDIS = !1;             /* SPI Moudle enable */
  DSPI(0).MCR.B.HALT = 1;              /* Stop transfer */
  DSPI(0).MODE.CTAR[0].B.DBR = DBR0;   /* Duty cycle is configurable */
  DSPI(0).MODE.CTAR[0].B.FMSZ = 0b0111;/* Frame size=8 */
  DSPI(0).MODE.CTAR[0].B.PCSSCK = 0b01;/* PCS to SCK delay scaler, 3 */
  DSPI(0).MODE.CTAR[0].B.CSSCK = 0b0100;/* PCS to SCK delay scaler, 32 */
  DSPI(0).MODE.CTAR[0].B.ASC = 0b0100; /* After SCK delay scaler */
  DSPI(0).MODE.CTAR[0].B.DT = 0b0100;  /* Delay after transfer scaler */
  DSPI(0).MODE.CTAR[0].B.CPOL = 0;
    /* Clock polarity, 0 means the inactive state of SCK is low, 1 means high */
  DSPI(0).MODE.CTAR[0].B.CPHA = 1;
          /* Clock phase, 0 means data is captured on the leading edge of SCK */
  DSPI(0).MODE.CTAR[0].B.LSBFE = 0;
                              /* LSB first enable, 0: MSB first; 1: LSB first */
  DSPI(0).MODE.CTAR[0].B.PBR = 0;      /* Baud rate prescaler */
  DSPI(0).MODE.CTAR[0].B.BR = 4;       /* Baud rate scaler */
  DSPI(0).MCR.B.HALT = 0;              /* Start transfer */
}

#endif

#if 1

/**************************************************************************
 * FUNCTION :    ec_spi1_init                                              *
 *                                                                         *
 * DESCRIPTION : Initialize DSPI1 module                                   *
 *                                                                         *
 * PARAMETERS :  None                                                      *
 *                                                                         *
 * RETURN:       None                                                      *
 ***************************************************************************/
void ec_spi1_init(void)
{
  DSPI(1).MCR.B.MSTR = 1;              /* SPI Mode,Master or Slave */
  DSPI(1).MCR.B.CONT_SCKE = 0;
  DSPI(1).MCR.B.DCONF = 0;
                         /* Configure DSPI_B in SPI, DSI or CSI configuration */
  DSPI(1).MCR.B.PCSIS = 0b11111111;
  DSPI(1).MCR.B.MDIS = !1;             /* SPI Moudle enable */
  DSPI(1).MCR.B.HALT = 1;              /* Stop transfer */
  DSPI(1).MODE.CTAR[0].B.DBR = DBR0;   /* Duty cycle is configurable*/
  DSPI(1).MODE.CTAR[0].B.FMSZ = 0b1111;/* Frame size=16 */
  DSPI(1).MODE.CTAR[0].B.PCSSCK = 0b01;/* PCS to SCK delay scaler, 3 */
  DSPI(1).MODE.CTAR[0].B.CSSCK = 0b0100;/* PCS to SCK delay scaler, 32 */
  DSPI(1).MODE.CTAR[0].B.ASC = 0b0100; /* After SCK delay scaler */
  DSPI(1).MODE.CTAR[0].B.DT = 0b0100;  /* Delay after transfer scaler */
  DSPI(1).MODE.CTAR[0].B.CPOL = 0;
    /* Clock polarity, 0 means the inactive state of SCK is low, 1 means high */
  DSPI(1).MODE.CTAR[0].B.CPHA = 1;
          /* Clock phase, 0 means data is captured on the leading edge of SCK */
  DSPI(1).MODE.CTAR[0].B.LSBFE = 0;
                              /* LSB first enable, 0: MSB first; 1: LSB first */
  DSPI(1).MODE.CTAR[0].B.PBR = 0;      /* Baud rate prescaler */
  DSPI(1).MODE.CTAR[0].B.BR = 3;       /* Baud rate scaler */
  DSPI(1).MCR.B.HALT = 0;              /* Start transfer */
}

#endif

#if (SPIAENABLE||SPIBENABLE)

/**************************************************************************
 * FUNCTION :    ec_spi_masterwriteread                                    *
 *                                                                         *
 * DESCRIPTION : SPI function in master mode                               *
 *                                                                         *
 * PARAMETERS :  -DSPIn: can be DSPI0, or DSPI1                 *
 *               -PCSn: can be PCS_0, PCS_1, PCS_2, PCS_3, PCS_4, PCS_5    *
 *               -Length: the number of data to be transmitted or received *
 *               -Tx_Buffer: a pointer pointing to the buffer used to save *
 *                the data transmitted by SPI                              *
 *               -Rx_Buffer: a pointer pointing to the buffer used to save *
 *                the data received by SPI                                 *
 *                                                                         *
 * RETURN:       None                                                      *
 ***************************************************************************/
void ec_spi_masterwriteread(uint8 DSPIn,uint8 PCSn,uint8 Length, uint16
  Tx_Buffer[], uint16 Rx_Buffer[])
{
  uint8 i = 0;
  for (i=0; i<Length; i++)             /* Transmit all SPI data frame */
  {
    switch (PCSn)                  /* Select PS accrording to hardware config */
    {
     case(PCS_0):
      DSPI(DSPIn).PUSHR.PUSHR.R = (uint32)(0x08010000|(uint16)Tx_Buffer[i]);
      break;

     case(PCS_1):
      DSPI(DSPIn).PUSHR.PUSHR.R = (uint32)(0x08020000|(uint16)Tx_Buffer[i]);
      break;

     case(PCS_2):
      DSPI(DSPIn).PUSHR.PUSHR.R = (uint32)(0x08040000|(uint16)Tx_Buffer[i]);
      break;

     case(PCS_3):
      DSPI(DSPIn).PUSHR.PUSHR.R = (uint32)(0x08080000|(uint16)Tx_Buffer[i]);
      break;

     case(PCS_4):
      DSPI(DSPIn).PUSHR.PUSHR.R = (uint32)(0x08100000|(uint16)Tx_Buffer[i]);
      break;

     case(PCS_5):
      DSPI(DSPIn).PUSHR.PUSHR.R = (uint32)(0x08200000|(uint16)Tx_Buffer[i]);
    }

    ec_spi_delay(DSPIn,100);
             /* Pop out of the delay function once the receive FIFO not empty */
    Rx_Buffer[i] = (uint16)DSPI(DSPIn).POPR.B.RXDATA;
    DSPI(DSPIn).SR.R = 0x90020000;
  }
}

#endif

/* File trailer for ECUCoder generated file spi.c.
 *
 * [EOF]
 */
