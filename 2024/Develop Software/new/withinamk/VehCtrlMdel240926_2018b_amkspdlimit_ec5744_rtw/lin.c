/*
 * Code generated for Simulink model VehCtrlMdel240926_2018b_amkspdlimit.
 *
 * FILE    : lin.c
 *
 * VERSION : 1.230
 *
 * DATE    : Sat Oct 12 03:28:34 2024
 *
 * Copyright 2011-2017 ECUCoder. All Rights Reserved.
 */

#include "lin.h"
#if 1
#define LIN0_COMBTX_DMACH              0
#define LIN0_COMBRX_DMACH              1

static uint8_t ec_lin0_txlock = 0;
LIN_INFO_tag ec_lin_dma_info;

#define LIN0_RXDMA_EN                  1
#define LIN0_COMBRX_DMACH              1

uint8_t LIN0_RXMSG_ACTIVE = 0;
static uint8_t ec_lin0_rxlock = 0;
static uint8_t ec_lin0_rxdata[64];
uint8_t ec_lin0_dstdata[64];

// LINA
#define LIN0_TXMSG_SIZE                64

uint8_t LIN0_TXMSG_SRCIDX = 0;
uint8_t LIN0_TXMSG_DSTIDX = 0;
LIN_TXMSG_T LIN0_TXMSG_BUF[LIN0_TXMSG_SIZE];
void ec_lin_delay(uint32 us)
{
  uint32 i,j;
  for (i=0; i<us; i++) {
    for (j=0; j<20; j++) {
      if (1 == LINFlexD_0.LINSR.B.DRF) {
        i = us+1;
        j = 21;
      }
    }
  }
}

void ec_lin_init(void)
{
  uint8 i;
  LINFlexD_0.LINCR1.R = 0x0081;        /* SLEEP=0, INIT=1 */
  while (0x1000 != (LINFlexD_0.LINSR.R & 0xF000)) {
  }

  LINFlexD_0.LINFBRR.R = 6;
  LINFlexD_0.LINIBRR.R = 260;
  LINFlexD_0.LINCR2.R = 0x40;   /* IOBE=1, Bit error resets LIN state machine */
  LINFlexD_0.LINTCSR.R = 0;           /* LIN timeout mode, no idle on timeout */
  LINFlexD_0.LINCR2.R = 0x00000800;
  ec_lin0_init_txdma();
  LINFlexD_0.LINCR1.R = 0x0390;

  //LINFlexD_0.LINCR2.B.DDRQ = 1;
}

void ec_lin_transmit(uint8 Tx_ID, uint8 Datalength, uint8 Tx_Data[], boolean
                     Checksum_Type)
{
  /* store the data in the message buffer BDR */
  switch (Datalength)
  {
   case 1:
    LINFlexD_0.BDRL.B.DATA0 = Tx_Data[0];
    break;

   case 2:
    LINFlexD_0.BDRL.B.DATA0 = Tx_Data[0];
    LINFlexD_0.BDRL.B.DATA1 = Tx_Data[1];
    break;

   case 3:
    LINFlexD_0.BDRL.B.DATA0 = Tx_Data[0];
    LINFlexD_0.BDRL.B.DATA1 = Tx_Data[1];
    LINFlexD_0.BDRL.B.DATA2 = Tx_Data[2];
    break;

   case 4:
    LINFlexD_0.BDRL.B.DATA0 = Tx_Data[0];
    LINFlexD_0.BDRL.B.DATA1 = Tx_Data[1];
    LINFlexD_0.BDRL.B.DATA2 = Tx_Data[2];
    LINFlexD_0.BDRL.B.DATA3 = Tx_Data[3];
    break;

   case 5:
    LINFlexD_0.BDRL.B.DATA0 = Tx_Data[0];
    LINFlexD_0.BDRL.B.DATA1 = Tx_Data[1];
    LINFlexD_0.BDRL.B.DATA2 = Tx_Data[2];
    LINFlexD_0.BDRL.B.DATA3 = Tx_Data[3];
    LINFlexD_0.BDRM.B.DATA4 = Tx_Data[4];
    break;

   case 6:
    LINFlexD_0.BDRL.B.DATA0 = Tx_Data[0];
    LINFlexD_0.BDRL.B.DATA1 = Tx_Data[1];
    LINFlexD_0.BDRL.B.DATA2 = Tx_Data[2];
    LINFlexD_0.BDRL.B.DATA3 = Tx_Data[3];
    LINFlexD_0.BDRM.B.DATA4 = Tx_Data[4];
    LINFlexD_0.BDRM.B.DATA5 = Tx_Data[5];
    break;

   case 7:
    LINFlexD_0.BDRL.B.DATA0 = Tx_Data[0];
    LINFlexD_0.BDRL.B.DATA1 = Tx_Data[1];
    LINFlexD_0.BDRL.B.DATA2 = Tx_Data[2];
    LINFlexD_0.BDRL.B.DATA3 = Tx_Data[3];
    LINFlexD_0.BDRM.B.DATA4 = Tx_Data[4];
    LINFlexD_0.BDRM.B.DATA5 = Tx_Data[5];
    LINFlexD_0.BDRM.B.DATA6 = Tx_Data[6];
    break;

   case 8:
    LINFlexD_0.BDRL.B.DATA0 = Tx_Data[0];
    LINFlexD_0.BDRL.B.DATA1 = Tx_Data[1];
    LINFlexD_0.BDRL.B.DATA2 = Tx_Data[2];
    LINFlexD_0.BDRL.B.DATA3 = Tx_Data[3];
    LINFlexD_0.BDRM.B.DATA4 = Tx_Data[4];
    LINFlexD_0.BDRM.B.DATA5 = Tx_Data[5];
    LINFlexD_0.BDRM.B.DATA6 = Tx_Data[6];
    LINFlexD_0.BDRM.B.DATA7 = Tx_Data[7];
    break;

   default :
    break;
  }

  LINFlexD_0.BIDR.B.DFL = Datalength - 1;/* DFL = Number of data bytes - 1 */
  LINFlexD_0.BIDR.B.DIR = 1;
                        /* LINFlexD transmits the data from the BDR registers */
  LINFlexD_0.BIDR.B.CCS = Checksum_Type;
                                /* 0: Enhanced Checksum ; 1: Classic Checksum */
  LINFlexD_0.BIDR.B.ID = Tx_ID; /* 0: Enhanced Checksum ; 1: Classic Checksum */
  LINFlexD_0.LINCR2.B.HTRQ = 1;        /* Trigger Frame transmission */

  /* wait until Master response to the LIN header has been sent successfully */
  while (0 == LINFlexD_0.LINSR.B.DTF) {
    /* track LIN Status for errors */
  }

  LINFlexD_0.LINSR.R = 0x0002;         /* clear the DTF bit */
}

void ec_lin_receive(uint8 Rx_ID, uint8 Datalength, uint8 Rx_Buffer[], boolean
                    Checksum_Type)
{
  LIN0_RXMSG_ACTIVE = 1;
  LINFlexD_0.BIDR.B.DFL = Datalength - 1;/* DFL = Number of data bytes - 1 */
  LINFlexD_0.BIDR.B.DIR = 0;
             /* LINFlexD receives the data and copy them in the BDR registers */
  LINFlexD_0.BIDR.B.CCS = Checksum_Type;
                                /* 0: Enhanced Checksum ; 1: Classic Checksum */
  LINFlexD_0.BIDR.B.ID = Rx_ID; /* 0: Enhanced Checksum ; 1: Classic Checksum */
  LINFlexD_0.LINCR2.B.HTRQ = 1;        /* Trigger Frame transmission */

  /* wait until Slave response to the LIN header has been receive successfully */
  ec_lin_delay(1000);

  /* read BDR registers */
  switch (Datalength)
  {
   case 1:
    Rx_Buffer[0] = LINFlexD_0.BDRL.B.DATA0;
    break;

   case 2:
    Rx_Buffer[0] = LINFlexD_0.BDRL.B.DATA0;
    Rx_Buffer[1] = LINFlexD_0.BDRL.B.DATA1;
    break;

   case 3:
    Rx_Buffer[0] = LINFlexD_0.BDRL.B.DATA0;
    Rx_Buffer[1] = LINFlexD_0.BDRL.B.DATA1;
    Rx_Buffer[2] = LINFlexD_0.BDRL.B.DATA2;
    break;

   case 4:
    Rx_Buffer[0] = LINFlexD_0.BDRL.B.DATA0;
    Rx_Buffer[1] = LINFlexD_0.BDRL.B.DATA1;
    Rx_Buffer[2] = LINFlexD_0.BDRL.B.DATA2;
    Rx_Buffer[3] = LINFlexD_0.BDRL.B.DATA3;
    break;

   case 5:
    Rx_Buffer[0] = LINFlexD_0.BDRL.B.DATA0;
    Rx_Buffer[1] = LINFlexD_0.BDRL.B.DATA1;
    Rx_Buffer[2] = LINFlexD_0.BDRL.B.DATA2;
    Rx_Buffer[3] = LINFlexD_0.BDRL.B.DATA3;
    Rx_Buffer[4] = LINFlexD_0.BDRM.B.DATA4;
    break;

   case 6:
    Rx_Buffer[0] = LINFlexD_0.BDRL.B.DATA0;
    Rx_Buffer[1] = LINFlexD_0.BDRL.B.DATA1;
    Rx_Buffer[2] = LINFlexD_0.BDRL.B.DATA2;
    Rx_Buffer[3] = LINFlexD_0.BDRL.B.DATA3;
    Rx_Buffer[4] = LINFlexD_0.BDRM.B.DATA4;
    Rx_Buffer[5] = LINFlexD_0.BDRM.B.DATA5;
    break;

   case 7:
    Rx_Buffer[0] = LINFlexD_0.BDRL.B.DATA0;
    Rx_Buffer[1] = LINFlexD_0.BDRL.B.DATA1;
    Rx_Buffer[2] = LINFlexD_0.BDRL.B.DATA2;
    Rx_Buffer[3] = LINFlexD_0.BDRL.B.DATA3;
    Rx_Buffer[4] = LINFlexD_0.BDRM.B.DATA4;
    Rx_Buffer[5] = LINFlexD_0.BDRM.B.DATA5;
    Rx_Buffer[6] = LINFlexD_0.BDRM.B.DATA6;
    break;

   case 8:
    Rx_Buffer[0] = LINFlexD_0.BDRL.B.DATA0;
    Rx_Buffer[1] = LINFlexD_0.BDRL.B.DATA1;
    Rx_Buffer[2] = LINFlexD_0.BDRL.B.DATA2;
    Rx_Buffer[3] = LINFlexD_0.BDRL.B.DATA3;
    Rx_Buffer[4] = LINFlexD_0.BDRM.B.DATA4;
    Rx_Buffer[5] = LINFlexD_0.BDRM.B.DATA5;
    Rx_Buffer[6] = LINFlexD_0.BDRM.B.DATA6;
    Rx_Buffer[7] = LINFlexD_0.BDRM.B.DATA7;
    break;

   default :
    break;
  }

  LINFlexD_0.LINSR.R = 0x0207;         /* clear RMB, HRF, DRF and DTF flags */
  LIN0_RXMSG_ACTIVE = 0;
}

/*****************************************************************************************
 *                        LIN-Tx DMAģʽ�¹���
 ****************************************************************************************/

/*
 *   LIN-Tx DMA Init
 */
void ec_lin0_init_txdma(void)
{
  XBAR_0.PORT[4].PRS.R = 0x03000021;   //����DMA��������0���ʾ���������ȼ�
  AIPS_0.MPRA.R |= 0x77777770;         //��������������������0���ж���дȨ��
  AIPS_1.MPRA.R |= 0x77777770;         //��������������������1���ж���дȨ��

  /* Init eDMA engine */
  DMA_0.CR.B.HALT= 1;                  //ֹͣDMA����

  /* cfg DMA_MUX for DMA channel 0 -- LIN0_COMBTX_DMACH */
  DMAMUX_0.CHCFG[LIN0_COMBTX_DMACH].R = 0x80 | 0x14;

  /* DMA_0 config for LIN_Tx */
  DMA_0.TCD[LIN0_COMBTX_DMACH].SADDR.R = (vuint32_t) &ec_lin_dma_info;//����Դ��ַ
  DMA_0.TCD[LIN0_COMBTX_DMACH].SOFF.R = 1;
  //����minor loop��������ÿ����һ�����ݴ���Դ��ַ�����������ֽ�Ϊ1 ��1�ֽ�  2:2�ֽ�  4:4�ֽ�DMA_0.TCD[LIN0_COMBTX_DMACH].ATTR.B.SMOD = 0;                          //��ѡ��Դ��ַȡģ
  DMA_0.TCD[LIN0_COMBTX_DMACH].ATTR.B.SSIZE = 0;//Դ�����ֽڳ���0:8-bit 1:16-bit 2:32-bit
  DMA_0.TCD[LIN0_COMBTX_DMACH].SLAST.R = -12;
        //���ݴ�������Դ��ַ���е�������ɺ�ĵ�ַ���ϸüĴ�����ֵΪ����Դ��ַ
  DMA_0.TCD[LIN0_COMBTX_DMACH].DADDR.R = (vuint32_t)&LINFlexD_0.BIDR.R;//����Ŀ�ĵ�ַ
  DMA_0.TCD[LIN0_COMBTX_DMACH].DOFF.R = 1;
          //����minor loop��������ÿ����һ�����ݴ���Ŀ�ĵ�ַ�����������ֽ�Ϊ��λ
  DMA_0.TCD[LIN0_COMBTX_DMACH].ATTR.B.DMOD = 0;//��ѡ��Ŀ�ĵ�ַȡģ
  DMA_0.TCD[LIN0_COMBTX_DMACH].ATTR.B.DSIZE = 0;//Ŀ�������ֳ���0:8-bit 1:16-bit 2:32-bit
  DMA_0.TCD[LIN0_COMBTX_DMACH].DLASTSGA.R = -12;
  //���ݴ�������Ŀ�ĵ�ַ���е�������ɺ�ĵ�ַ���ϸüĴ�����ֵΪ����Ŀ�ĵ�ַ,���ֽ�Ϊ��λ
  DMA_0.TCD[LIN0_COMBTX_DMACH].NBYTES.MLNO.B.NBYTES = 1;//ѡ��minor loop������ֽ���Ŀ
  DMA_0.TCD[LIN0_COMBTX_DMACH].CSR.B.DREQ = 1;
  DMA_0.TCD[LIN0_COMBTX_DMACH].BITER.ELINKNO.B.ELINK = 0;//minor loop��ɺ�ѡ��ͨ������
  DMA_0.TCD[LIN0_COMBTX_DMACH].BITER.ELINKNO.B.BITER = 12;//���ÿ�ʼminor loop��Ŀ
  DMA_0.TCD[LIN0_COMBTX_DMACH].CITER.ELINKNO.B.ELINK = 0;//minor loop��ɺ�ѡ��ͨ������
  DMA_0.TCD[LIN0_COMBTX_DMACH].CITER.ELINKNO.B.CITER = 12;
        //���õ�ǰminor loop��Ŀ����ʼ��ʱӦ��֤CITER��BITER��minor loop��Ŀ���
  DMA_0.TCD[LIN0_COMBTX_DMACH].CSR.B.INTHALF = 0;
  DMA_0.TCD[LIN0_COMBTX_DMACH].CSR.B.INTMAJOR = 0;
  DMA_0.TCD[LIN0_COMBTX_DMACH].CSR.B.MAJORELINK = 0;
  DMA_0.TCD[LIN0_COMBTX_DMACH].CSR.B.ESG = 0;
  DMA_0.TCD[LIN0_COMBTX_DMACH].CSR.B.BWC = 0;
  DMA_0.TCD[LIN0_COMBTX_DMACH].CSR.B.DONE = 0;
  DMA_0.TCD[LIN0_COMBTX_DMACH].CSR.B.ACTIVE = 0;
  DMA_0.TCD[LIN0_COMBTX_DMACH].CSR.B.START = 0;
  LINFlexD_0.DMATXE.R = 0x00000001;    //enable DMA (DMA_TX_CH_NUM = 1)
  DMA_0.CR.B.ERGA= 1;                  //ѡ�������ȼ�����Ϊ�ػ�����
  DMA_0.CR.B.ERCA= 1;                  //ѡ������ͨ�����ȼ�����Ϊ�ػ���
  DMA_0.CR.B.HALT= 0;                  //��ʼDMA����
}

/*
 * LIN-Tx DMA����ͨ������
 */
void ec_lin0_transmit_tcd(uint8_t Datalength)
{
  uint8 numbyte;
  if (Datalength < 5) {
    numbyte = 4+4;
  } else {
    numbyte = 4+8;
  }

  DMA_0.TCD[LIN0_COMBTX_DMACH].CITER.ELINKNO.B.CITER = numbyte;
        //���õ�ǰminor loop��Ŀ����ʼ��ʱӦ��֤CITER��BITER��minor loop��Ŀ���
  DMA_0.TCD[LIN0_COMBTX_DMACH].BITER.ELINKNO.B.BITER = numbyte;//���ÿ�ʼminor loop��Ŀ
  DMA_0.TCD[LIN0_COMBTX_DMACH].NBYTES.MLNO.B.NBYTES = 1;//ѡ��minor loop������ֽ���Ŀ
  DMA_0.TCD[LIN0_COMBTX_DMACH].SLAST.R = -numbyte;
        //���ݴ�������Դ��ַ���е�������ɺ�ĵ�ַ���ϸüĴ�����ֵΪ����Դ��ַ
  DMA_0.TCD[LIN0_COMBTX_DMACH].DLASTSGA.R = -numbyte;
  //���ݴ�������Ŀ�ĵ�ַ���е�������ɺ�ĵ�ַ���ϸüĴ�����ֵΪ����Ŀ�ĵ�ַ,���ֽ�Ϊ��λ
  DMA_0.TCD[LIN0_COMBTX_DMACH].CSR.B.DREQ = 1;
  DMA_0.TCD[LIN0_COMBTX_DMACH].CSR.B.DONE = 0;
  DMA_0.TCD[LIN0_COMBTX_DMACH].CSR.B.ACTIVE = 0;
  DMA_0.SERQ.B.SERQ = LIN0_COMBTX_DMACH;
  DMA_0.SERQ.B.NOP = 0;
}

/*
 * LIN-Tx DMA Ӧ�ò�ӿ�
 */
static inline void LIN0_TXMSG_SRCIDX_INC(void)
{
  LIN0_TXMSG_SRCIDX++;
  if (LIN0_TXMSG_SRCIDX>=LIN0_TXMSG_SIZE) {
    LIN0_TXMSG_SRCIDX = 0;
  }
}

static inline void LIN0_TXMSG_DSTIDX_INC(void)
{
  LIN0_TXMSG_DSTIDX++;
  if (LIN0_TXMSG_DSTIDX>=LIN0_TXMSG_SIZE) {
    LIN0_TXMSG_DSTIDX = 0;
  }
}

void Lin0_PushTxMsg(uint8 Tx_ID, uint8 Datalength, uint8 Tx_Data[], boolean
                    Checksum_Type)
{
  uint8_t i = 0;
  pLIN_TXMSG_T pTxMsg = &LIN0_TXMSG_BUF[LIN0_TXMSG_DSTIDX];
  pTxMsg->BufId = Tx_ID;
  pTxMsg->Length = Datalength;
  pTxMsg->ChecksumType = Checksum_Type;
  for (i=0; i<Datalength; i++) {
    pTxMsg->Data[i] = Tx_Data[i];
  }

  LIN0_TXMSG_DSTIDX_INC();
}

/*
 * LIN-Tx DMA �ײ㷢��
 */
static uint8_t is_lin0_txlock(void)
{
  if (1==DMA_0.TCD[LIN0_COMBTX_DMACH].CSR.B.DONE) {
    ec_lin0_txlock = 0;
  }

  return ec_lin0_txlock;
}

uint8_t is_lin0_txbusy(void)
{
  if (2==LINFlexD_0.LINSR.B.LINS && 0==is_lin0_txlock()) {
    return 0;
  }

  return 1;
}

void ec_lin0_transmit_dma(uint8 Tx_ID, uint8 Datalength, uint8 Tx_Data[],
  boolean Checksum_Type)
{
  uint8_t i;
  if (0 == is_lin0_txlock()) {
    ec_lin0_txlock = 1;
    ec_lin_dma_info.BIDR.B.DFL = Datalength - 1;/* DFL = Number of data bytes - 1 */
    ec_lin_dma_info.BIDR.B.DIR = 1;
                        /* LINFlexD transmits the data from the BDR registers */
    ec_lin_dma_info.BIDR.B.CCS = Checksum_Type;
                                /* 0: Enhanced Checksum ; 1: Classic Checksum */
    ec_lin_dma_info.BIDR.B.ID = Tx_ID;
    ec_lin_dma_info.BDRL.R = 0;
    ec_lin_dma_info.BDRM.R = 0;
    if (Datalength < 5) {
      for (i=0; i<Datalength; i++) {
        ec_lin_dma_info.BDRL.R += ((uint32)Tx_Data[i] << i*8);
      }
    } else {
      for (i=0; i<Datalength; i++) {
        if (i<4) {
          ec_lin_dma_info.BDRL.R += ((uint32)Tx_Data[i] << i*8);
        } else {
          ec_lin_dma_info.BDRM.R += ((uint32)Tx_Data[i] << (i-4)*8);
        }
      }
    }

    ec_lin0_transmit_tcd(Datalength);
  }
}

void Lin0_TxBackground(void)
{
  // LIN0
  if (0==is_lin0_txbusy()&&0==LIN0_RXMSG_ACTIVE) {
    if (LIN0_TXMSG_SRCIDX != LIN0_TXMSG_DSTIDX) {
      pLIN_TXMSG_T pTxMsg = &LIN0_TXMSG_BUF[LIN0_TXMSG_SRCIDX];
      ec_lin0_transmit_dma(pTxMsg->BufId, pTxMsg->Length, pTxMsg->Data,
                           pTxMsg->ChecksumType);
      LIN0_TXMSG_SRCIDX_INC();
    }
  }
}

void Lin0_Background(void)
{
  Lin0_TxBackground();

  //LinNm_RxBackground();
}

#if 0

void ec_scia_init_rxdma()
{
  //EDMA.TCD[ESCIA_COMBRX_DMACH].SADDR = (vuint32_t) &ec_scia_rxdata;  /* Load address of source data */
  EDMA.TCD[ESCIA_COMBRX_DMACH].SADDR = (vuint32_t) &(ESCI(0).DR.R)+1;/* Load address of source data */
  EDMA.TCD[ESCIA_COMBRX_DMACH].SSIZE = 0;
  EDMA.TCD[ESCIA_COMBRX_DMACH].SOFF = 0;
  EDMA.TCD[ESCIA_COMBRX_DMACH].SLAST = 0;
  EDMA.TCD[ESCIA_COMBRX_DMACH].SMOD = 0;
  EDMA.TCD[ESCIA_COMBRX_DMACH].DADDR = (vuint32_t) &ec_scia_rxdata;/* Load address of destination */

  //EDMA.TCD[ESCIA_COMBRX_DMACH].DADDR = (vuint32_t) &ec_scia_dstdata; /* Load address of destination */
  EDMA.TCD[ESCIA_COMBRX_DMACH].DSIZE = 0;
  EDMA.TCD[ESCIA_COMBRX_DMACH].DOFF = 1;
  EDMA.TCD[ESCIA_COMBRX_DMACH].DLAST_SGA = 0;
  EDMA.TCD[ESCIA_COMBRX_DMACH].DMOD = 0;
  EDMA.TCD[ESCIA_COMBRX_DMACH].NBYTES = 1;/* Transfer 1 byte per minor loop */
  EDMA.TCD[ESCIA_COMBRX_DMACH].BITER = 12;
  EDMA.TCD[ESCIA_COMBRX_DMACH].CITER = 12;/* Initialize current iteraction count */
  EDMA.TCD[ESCIA_COMBRX_DMACH].D_REQ = 1;
                                    /* Disable channel when major loop is done*/
  EDMA.TCD[ESCIA_COMBRX_DMACH].INT_HALF = 0;
  EDMA.TCD[ESCIA_COMBRX_DMACH].INT_MAJ = 0;
  EDMA.TCD[ESCIA_COMBRX_DMACH].CITERE_LINK = 0;/* Linking is not used */
  EDMA.TCD[ESCIA_COMBRX_DMACH].BITERE_LINK = 0;
  EDMA.TCD[ESCIA_COMBRX_DMACH].MAJORE_LINK = 0;
  EDMA.TCD[ESCIA_COMBRX_DMACH].E_SG = 0;
  EDMA.TCD[ESCIA_COMBRX_DMACH].BWC = 0;
                                      /* Default bandwidth control- no stalls */
  EDMA.TCD[ESCIA_COMBRX_DMACH].START = 0;/* Initialize status flags */
  EDMA.TCD[ESCIA_COMBRX_DMACH].DONE = 0;
  EDMA.TCD[ESCIA_COMBRX_DMACH].ACTIVE = 0;
  ESCI(0).CR2.B.RXDMA = 1;
}

#endif
#endif

/* File trailer for ECUCoder generated file lin.c.
 *
 * [EOF]
 */
