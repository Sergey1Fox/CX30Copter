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
    {0x50, 0},    /* ACTIVATE command */
    {0x53, 0},    /* LOCK/UNLOCK */
    
    /* Step 5-9: Write registers 0x20-0x27 (pipe settings) */
    {0x20, 0},    /* Write CONFIG */
    {0x08, 0},    /* CONFIG value: PRIM_RX=0, PWR_UP=1 */
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
    {0x07, 0},    /* Clear interrupts */
    {0x28, 0},    /* Write OBSERVE_TX */
    {0x00, 0},    /* OBSERVE_TX value */
    {0x29, 0},    /* Write RPD */
    {0x00, 0},    /* RPD value */
    
    /* Step 34-37: RX pipe 0 payload width */
    {0x2C, 0},    /* Write RX_PW_P0 */
    {0xC3, 0},    /* 8-byte payload */
    {0x2D, 0},    /* Write RX_PW_P1 */
    {0xC4, 0},    /* 8-byte payload */
    {0x2E, 0},    /* Write RX_PW_P2 */
    {0xC5, 0},    /* 8-byte payload */
    {0x2F, 0},    /* Write RX_PW_P3 */
    {0xC6, 0},    /* 8-byte payload */
    
    /* Step 42-55: Address setup */
    {0x31, 0},    /* Write TX_ADDR */
    {0x20, 0},    /* Address byte 0 */
    {0x32, 0},    /* Write TX_ADDR cont */
    {0x20, 0},    /* Address byte 1 */
    {0x33, 0},    /* Write TX_ADDR cont */
    {0x20, 0},    /* Address byte 2 */
    {0x34, 0},    /* Write TX_ADDR cont */
    {0x20, 0},    /* Address byte 3 */
    {0x35, 0},    /* Write TX_ADDR cont */
    {0x20, 0},    /* Address byte 4 */
    {0x36, 0},    /* Write RX_ADDR_P0 */
    {0x20, 0},    /* Address byte 0 */
    {0x37, 0},    /* Write RX_ADDR_P0 cont */
    {0x00, 0},    /* Address byte 1 */
    
    /* Step 56-67: RX pipe addresses */
    {0x2A, 0},    /* Write RX_ADDR_P1 */
    {0x65, 0},    /* Address byte 0 */
    {0x65, 0},    /* Address byte 1 */
    {0x65, 0},    /* Address byte 2 */
    {0x65, 0},    /* Address byte 3 */
    {0x65, 0},    /* Address byte 4 */
    {0x30, 0},    /* Write RX_ADDR_P0 */
    {0x65, 0},    /* Address byte 0 */
    {0x65, 0},    /* Address byte 1 */
    {0x65, 0},    /* Address byte 2 */
    {0x65, 0},    /* Address byte 3 */
    {0x65, 0},    /* Address byte 4 */
    
    /* Step 68-75: Final config */
    {0x1D, 0},    /* Write FEATURE */
    {0xA5, 0},    /* FEATURE value */
    {0x50, 0},    /* ACTIVATE */
    {0x73, 0},    /* ACTIVATE data */
    {0x3C, 0},    /* Write ?? */
    {0x3F, 0},    /* Value */
    {0x3D, 0},    /* Write ?? */
    {0x07, 0},    /* Value */
    
    /* Step 76-79: Final activate */
    {0x07, 0},
    {0xA5, 0},
    {0x50, 0},
    {0x53, 0},
    
    /* Step 80-84: Write register 0x20 */
    {0x20, 0},
    {0x40, 0},    /* CONFIG: PWR_UP=1 */
    {0x4B, 0},    /* Additional config */
    {0x01, 0},
    {0xE2, 0},    /* FLUSH_RX */
    
    /* Step 85-89: Write register 0x21 */
    {0x21, 0},
    {0xC0, 0},
    {0x4B, 0},
    {0x00, 0},
    {0x00, 0},
    
    /* Step 90-94: Write register 0x22 */
    {0x22, 0},
    {0xD0, 0},
    {0xFC, 0},
    {0x8C, 0},
    {0x02, 0},
    
    /* Step 95-99: Write register 0x23 */
    {0x23, 0},
    {0x99, 0},
    {0x00, 0},
    {0x39, 0},
    {0x21, 0},
    
    /* Step 100-104: Write register 0x24 */
    {0x24, 0},
    {0xF9, 0},
    {0x96, 0},
    {0x82, 0},
    {0x1B, 0},
    
    /* Step 105-109: Write register 0x25 */
    {0x25, 0},
    {0x24, 0},
    {0x06, 0},
    {0x0F, 0},
    {0xA6, 0},
    
    /* Step 110-114: Write register 0x2C */
    {0x2C, 0},
    {0x00, 0},
    {0x12, 0},
    {0x73, 0},
    {0x00, 0},
    
    /* Step 115-119: Write register 0x2D */
    {0x2D, 0},
    {0x36, 0},
    {0xB4, 0},
    {0x80, 0},
    {0x00, 0},
    
    /* Step 120-131: Write register 0x2E */
    {0x2E, 0},
    {0x41, 0},
    {0x10, 0},
    {0x04, 0},
    {0x82, 0},
    {0x20, 0},
    {0x08, 0},
    {0x08, 0},
    {0xF2, 0},
    {0x7D, 0},
    {0xEF, 0},
    {0xFF, 0},
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
