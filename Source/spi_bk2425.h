/**
  ******************************************************************************
  * @file    spi_bk2425.h
  * @brief   SPI driver for BK2425 RF transceiver
  *          SPI Master, 4MHz clock
  ******************************************************************************
  */

#ifndef __SPI_BK2425_H
#define __SPI_BK2425_H

#include "stm8s.h"

/* SPI Command Definitions */
#define BK_CMD_READ_REG     0x00    /* Read register */
#define BK_CMD_WRITE_REG    0x20    /* Write register */
#define BK_CMD_RX_PAYLOAD_WIDTH   0x60    /* Read RX payload width*/
#define BK_CMD_RX_PAYLOAD   0x61    /* Read RX payload */
#define BK_CMD_TX_PAYLOAD   0xA0    /* Write TX payload */
#define BK_CMD_FLUSH_RX     0xE2    /* Flush RX FIFO */
#define BK_CMD_FLUSH_TX     0xE1    /* Flush TX FIFO */
#define BK_CMD_ACTIVATE     0x50    /* Activate features */
#define BK_CMD_TOGGLE_BANK  0x53    /* Toggles the register bank */
#define BK_CMD_FEATURES     0x73    /* Activate features */
#define BK_CMD_NOP          0xFF    /* No operation */

/* BK2425 Register Addresses */
#define BK_REG_CONFIG       0x00
#define BK_REG_EN_AA        0x01
#define BK_REG_EN_RXADDR    0x02
#define BK_REG_SETUP_AW     0x03
#define BK_REG_SETUP_RETR   0x04
#define BK_REG_RF_CH        0x05
#define BK_REG_RF_SETUP     0x06
#define BK_REG_STATUS       0x07
#define BK_REG_OBSERVE_TX   0x08
#define BK_REG_CD           0x09  /* Carrier Detect */
#define BK_REG_RX_ADDR_P0   0x0A
#define BK_REG_RX_ADDR_P1   0x0B
#define BK_REG_TX_ADDR      0x10
#define BK_REG_RX_PW_P0     0x11
#define BK_REG_RX_PW_P1     0x12
#define BK_REG_FIFO_STATUS  0x17
#define BK_REG_DYNPAYLOAD   0x1C
#define BK_REG_FEATURE      0x1D

/* BK2425 RX Payload */
typedef struct {
    uint8_t left_stick_y;     /* 0x80 default, 0x00-0xFF, vertical speed */
    uint8_t left_stick_x;     /* 0x80 default, 0x00-0xFF, afterburner */
    uint8_t left_rocker;      /* 0x40 default, 0x0E-0x72, pitch adjust */
    uint8_t right_stick_y;    /* 0x80 default, 0x60-0xA0, pitch control */
    uint8_t right_stick_x;    /* 0x80 default, 0x60-0xA0, roll control */
    uint8_t reserved;         /* 0x40 default */
    uint8_t right_rocker;     /* 0x40 default, 0x0E-0x72, roll adjust */
    uint8_t buttons;          /* 0x00 default, 0x40 = left upper 2nd button */
} BK_Payload_t;

/* External state */
extern volatile uint8_t bk_payload_received;
extern volatile uint16_t bk_no_payload_count;
extern BK_Payload_t bk_payload;

void SPI_BK_Init(void);
uint8_t SPI_BK_ReadReg(uint8_t reg);
void SPI_BK_WriteReg(uint8_t reg, uint8_t value);
uint8_t SPI_BK_ReadPayload(void);
void SPI_BK_FlushRX(void);
void SPI_BK_FlushTX(void);
uint8_t SPI_BK_ReadStatus(void);

#endif /* __SPI_BK2425_H */
