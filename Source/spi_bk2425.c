/**
  ******************************************************************************
  * @file    spi_bk2425.c
  * @brief   SPI driver for BK2425 RF transceiver
  ******************************************************************************
  */

#include "spi_bk2425.h"
#include "gpio_config.h"

/* BK2425 initialization sequence from BK_Init.txt */
/* Format: { MOSI_data } - the MISO response is ignored during init */
typedef struct {
    uint8_t mosi;
    uint16_t delay_us;  /* Delay after transaction (if needed) */
} BK_InitStep_t;

/* Parsed initialization sequence from BK_Init.txt */
static const BK_InitStep_t bk_init_sequence[] = {
    /* Step 1-4: Activate and unlock */
    {0x07, 0},    /* NOP */
    {0xA5, 0},    /* Part of activate sequence */
    
    /* Step 5-9: Write registers 0x20-0x27 (pipe settings) */
    {0x20, 0},    /* Write CONFIG */
    {0x0B, 0},    /* CONFIG value: EN_CRC=1, PRIM_RX=1, PWR_UP=1 */
    {0x21, 0},    /* Write EN_AA */
    {0x3F, 0},    /* EN_AA value: Enable auto ACK all pipes */
    {0x22, 0},    /* Write EN_RXADDR */
    {0x3F, 0},    /* EN_RXADDR value: Enable all pipes */
    {0x23, 0},    /* Write SETUP_AW */
    {0x03, 0},    /* 5-byte address width */
    {0x24, 0},    /* Write SETUP_RETR */
    {0x1F, 0},    /* ARD=500us, ARC=15 retries */
    {0x25, 0},    /* Write RF_CH */
    {0x3C, 0},    /* Channel 60 */
    {0x26, 0},    /* Write RF_SETUP */
    {0x07, 0},    /* 0dBm, 1Mbps */
    {0x27, 0},    /* Write STATUS */
    {0x07, 0},    /* Clear interrupts ??? */
    {0x28, 0},    /* Write OBSERVE_TX */
    {0x00, 0},    /* OBSERVE_TX value ??? */
    {0x29, 0},    /* Write RPD */
    {0x00, 0},    /* RPD value ??? */
    
    /* Step 34-37: Address setup */
    {0x2C, 0},    /* Write RX_ADDR_P2 */
    {0xC3, 0},    /* Receive address data pipe 2 */
    {0x2D, 0},    /* Write RX_ADDR_P3 */
    {0xC4, 0},    /* Receive address data pipe 3 */
    {0x2E, 0},    /* Write RX_ADDR_P4 */
    {0xC5, 0},    /* Receive address data pipe 4 */
    {0x2F, 0},    /* Write RX_ADDR_P5 */
    {0xC6, 0},    /* Receive address data pipe 5 */
    
    /* Step 42-55: RX pipes payload width */
    {0x31, 0},    /* Write RX_PW_P0 */
    {0x20, 0},    /* Bytes in RX payload data pipe 0 */
    {0x32, 0},    /* Write RX_PW_P1 */
    {0x20, 0},    /* Bytes in RX payload data pipe 1 */
    {0x33, 0},    /* Write RX_PW_P2 */
    {0x20, 0},    /* Bytes in RX payload data pipe 2 */
    {0x34, 0},    /* Write RX_PW_P3 */
    {0x20, 0},    /* Bytes in RX payload data pipe 3 */
    {0x35, 0},    /* Write RX_PW_P4 */
    {0x20, 0},    /* Bytes in RX payload data pipe 4 */
    {0x36, 0},    /* Write RX_PW_P5 */
    {0x20, 0},    /* Bytes in RX payload data pipe 5 */
    {0x37, 0},    /* Write FIFO_STATUS */
    {0x00, 0},    /* ??? */
    
    /* Step 56-67: RX/TX pipe addresses */
    {0x2A, 0},    /* Write RX_ADDR_P0 */
    {0x65, 0},    /* Address byte 0 */
    {0x65, 0},    /* Address byte 1 */
    {0x65, 0},    /* Address byte 2 */
    {0x65, 0},    /* Address byte 3 */
    {0x65, 0},    /* Address byte 4 */
    {0x30, 0},    /* Write TX_ADDR */
    {0x65, 0},    /* Address byte 0 */
    {0x65, 0},    /* Address byte 1 */
    {0x65, 0},    /* Address byte 2 */
    {0x65, 0},    /* Address byte 3 */
    {0x65, 0},    /* Address byte 4 */
    
    /* Step 68-75: Final config */
    {0x1D, 0},    /* Read FEATURE */
    {0xA5, 0},    /* Get FEATURE value */
    {0x50, 0},    /* ACTIVATE features */
    {0x73, 0},    /* R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK features */
    {0x3C, 0},    /* Write DYNPD */
    {0x3F, 0},    /* Enable dynamic payload length for all pipes */
    {0x3D, 0},    /* Write FEATURE */
    {0x07, 0},    /* Enables Dynamic Payload Length, Enables Payload with ACK, Enables the W_TX_PAYLOAD_NOACK command */
    
    /* Step 76-79: Final activate */
    {0x07, 0},    /* Read STATUS */
    {0xA5, 0},
    {0x50, 0},    /* Switch to Register Bank 1 */
    {0x53, 0},
    
    /* For register 0 to 8 at bank 1, the byte
       order is inversed that the MSB byte is R/W before LSB byte. */
    /* Step 80-84: Write register 0x00  Bank 1 */
    {0x20, 0},
    {0x40, 0},    /* Must write with 0x404B01E2 */
    {0x4B, 0}, 
    {0x01, 0},
    {0xE2, 0},    
    /* Step 85-89: Write register 0x01 Bank 1 */
    {0x21, 0},
    {0xC0, 0},    /* Must write with 0xC04B0000 */
    {0x4B, 0},
    {0x00, 0},
    {0x00, 0},    
    /* Step 90-94: Write register 0x02 Bank 1 */
    {0x22, 0},
    {0xD0, 0},    /* Must write with 0xD0FC8C02 */
    {0xFC, 0},
    {0x8C, 0},
    {0x02, 0},    
    /* Step 95-99: Write register 0x03 Bank 1 */
    {0x23, 0},
    {0x99, 0},    /* Must write with 0x99003921 */
    {0x00, 0},
    {0x39, 0},
    {0x21, 0},

    /* Step 100-104: Write register 0x04 Bank 1 */
    {0x24, 0},
    {0xF9, 0},    /* 1Msps 0xF996821B */
    {0x96, 0},    /* 2Msps 0xF99682DB */
    {0x82, 0},    /* 250ksps 0xF9968ADB */
    {0x1B, 0},
    
    /* Step 105-109: Write register 0x05 Bank 1 */
    {0x25, 0},
    {0x24, 0},    /* 1Msps 0x24060FA6 (Disable RSSI) */
    {0x06, 0},
    {0x0F, 0},
    {0xA6, 0},
    
    /* Step 110-114: Write register 0x0C Bank 1 */
    {0x2C, 0},
    {0x00, 0},    /* Please initialize with 0x05731200 For 120us mode 0x00731200 */
    {0x12, 0},
    {0x73, 0},
    {0x00, 0},
    
    /* Step 115-119: Write register NEW_FEATURE 0x0D Bank 1 */
    {0x2D, 0},
    {0x36, 0},    /* Initialize with 0x0080B436 */
    {0xB4, 0},
    {0x80, 0},
    {0x00, 0},
    
    /* Step 120-131: Write register RAMP 0x0E Bank 1 */
    {0x2E, 0},
    {0x41, 0},    /* Ramp curve */
    {0x10, 0},    /* Please write with */
    {0x04, 0},    /* 0xFFFFFEF7CF208104082041 */
    {0x82, 0},
    {0x20, 0},
    {0x08, 0},
    {0x08, 0},
    {0xF2, 0},
    {0x7D, 0},
    {0xEF, 0},
    {0xFF, 0},

    {0x50, 0},    /* Switch to Register Bank 0 */
    {0x53, 0},
    {0xE2, 0},    /* Flush RX FIFO */
    {0x00, 0},
    {0x27, 0},    /* Write STATUS */
    {0x0E, 0},
};

#define BK_INIT_STEPS (sizeof(bk_init_sequence) / sizeof(bk_init_sequence[0]))

/* External state */
volatile uint8_t bk_payload_received = 0;
volatile uint16_t bk_no_payload_count = 0;
BK_Payload_t bk_payload;

static void SPI_WaitBusy(void)
{
    while (SPI_GetFlagStatus(SPI_FLAG_BSY));
}

static void SPI_Select(void)
{
    SPI_WaitBusy();
    GPIO_WriteLow(GPIOE, GPIO_PIN_5);  /* SS low */
}

static void SPI_Deselect(void)
{
    GPIO_WriteHigh(GPIOE, GPIO_PIN_5);  /* SS high */
    SPI_WaitBusy();
}

static uint8_t SPI_Transfer(uint8_t data)
{
    SPI_SendData(data);
    while (!SPI_GetFlagStatus(SPI_FLAG_RXNE));
    return SPI_ReceiveData();
}

void SPI_BK_Init(void)
{
    uint16_t i;
    
    /* Enable SPI clock */
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, ENABLE);
    
    /* SPI Init: Master, 4MHz, CPOL=0, CPHA=0 */
    SPI_DeInit();
    SPI_Init(SPI_FIRSTBIT_MSB,
             SPI_BAUDRATEPRESCALER_4,  /* 16MHz/4 = 4MHz */
             SPI_MODE_MASTER,
             SPI_CLOCKPOLARITY_LOW,
             SPI_CLOCKPHASE_1EDGE,
             SPI_DATADIRECTION_2LINES_FULLDUPLEX,
             SPI_NSS_SOFT,
             0x07);
    SPI_Cmd(ENABLE);
    
    /* Send initialization sequence */
    SPI_Select();
    for (i = 0; i < BK_INIT_STEPS; i++) {
        SPI_Transfer(bk_init_sequence[i].mosi);
        if (bk_init_sequence[i].delay_us > 0) {
            /* Delay if needed */
            uint16_t j;
            for (j = 0; j < bk_init_sequence[i].delay_us; j++) {
#if defined(_SDCC_)
                __asm__("nop");
#elif defined(_IAR_)
                asm("nop");
#endif
            }
        }
    }
    SPI_Deselect();
    
    /* Small delay after init */
    {
        volatile uint32_t delay;
        for (delay = 0; delay < 10000; delay++);
    }
    
    /* Read STATUS to verify */
    SPI_Select();
    SPI_Transfer(BK_CMD_NOP);
    SPI_Deselect();
}

uint8_t SPI_BK_ReadReg(uint8_t reg)
{
    uint8_t value;
    
    SPI_Select();
    SPI_Transfer(reg);  /* Read command = register address */
    value = SPI_Transfer(BK_CMD_NOP);
    SPI_Deselect();
    
    return value;
}

void SPI_BK_WriteReg(uint8_t reg, uint8_t value)
{
    SPI_Select();
    SPI_Transfer(reg | BK_CMD_WRITE_REG);
    SPI_Transfer(value);
    SPI_Deselect();
}

uint8_t SPI_BK_ReadPayload(void)
{
    uint8_t i;
    uint8_t *data = (uint8_t*)&bk_payload;
    uint8_t status;
    uint8_t success = 0;
    
    /* Check STATUS register for RX_DR bit */
    SPI_Select();
    status = SPI_Transfer(BK_CMD_NOP);
    SPI_Deselect();
    
    /* Check if RX FIFO has data */
    if (status & 0x40) {  /* RX_DR bit */
        SPI_Select();
        SPI_Transfer(BK_CMD_RX_PAYLOAD);
        
        /* Read 8 bytes */
        for (i = 0; i < 8; i++) {
            data[i] = SPI_Transfer(BK_CMD_NOP);
        }
        SPI_Deselect();
        
        bk_payload_received = 1;
        bk_no_payload_count = 0;
        success = 1;
        
        /* Clear RX_DR interrupt */
        SPI_BK_WriteReg(BK_REG_STATUS, status & ~0x40);
    } else {
        bk_no_payload_count++;
        if (bk_no_payload_count > 10) {
            bk_payload_received = 0;
        }
    }
    
    return success;
}

void SPI_BK_FlushRX(void)
{
    SPI_Select();
    SPI_Transfer(BK_CMD_FLUSH_RX);
    SPI_Deselect();
}

void SPI_BK_FlushTX(void)
{
    SPI_Select();
    SPI_Transfer(BK_CMD_FLUSH_TX);
    SPI_Deselect();
}

uint8_t SPI_BK_ReadStatus(void)
{
    uint8_t status;
    
    SPI_Select();
    status = SPI_Transfer(BK_CMD_NOP);
    SPI_Deselect();
    
    return status;
}
