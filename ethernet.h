/*
 * ethernet.h
 *
 *  Created on: Apr 24, 2015
 *      Author: lhep
 */

#ifndef INC_ETHERNET_H_
#define INC_ETHERNET_H_

#define LPC_SSP           LPC_SSP0
#define SSP_IRQ           SSP0_IRQn
#define LPC_GPDMA_SSP_TX  GPDMA_CONN_SSP0_Tx
#define LPC_GPDMA_SSP_RX  GPDMA_CONN_SSP0_Rx
#define SSPIRQHANDLER SSP0_IRQHandler

#define BUFFER_SIZE                         (0x03)
#define SSP_DATA_BITS                       (SSP_BITS_8)
#define SSP_DATA_BIT_NUM(databits)          (databits+1)
#define SSP_DATA_BYTES(databits)            (((databits) > SSP_BITS_8) ? 2:1)
#define SSP_LO_BYTE_MSK(databits)           ((SSP_DATA_BYTES(databits) > 1) ? 0xFF:(0xFF>>(8-SSP_DATA_BIT_NUM(databits))))
#define SSP_HI_BYTE_MSK(databits)           ((SSP_DATA_BYTES(databits) > 1) ? (0xFF>>(16-SSP_DATA_BIT_NUM(databits))):0)

#define SSP_MODE_SEL                        (0x31)
#define SSP_TRANSFER_MODE_SEL               (0x32)
#define SSP_MASTER_MODE_SEL                 (0x31)
#define SSP_SLAVE_MODE_SEL                  (0x32)
#define SSP_POLLING_SEL                     (0x31)
#define SSP_INTERRUPT_SEL                   (0x32)
#define SSP_DMA_SEL                         (0x33)

#define ENET_NUM_TX_DESC 4
#define ENET_NUM_RX_DESC 4

uint8_t macaddr[6], dstmacaddr[6], *workbuff;
/* Transmit/receive buffers and indices */
uint8_t TXBuffer[ENET_NUM_TX_DESC][EMAC_ETH_MAX_FLEN];
uint8_t RXBuffer[ENET_NUM_RX_DESC][EMAC_ETH_MAX_FLEN];
int32_t rxFill, rxGet, rxAvail, rxNumDescs;
int32_t txFill, txGet, txUsed, txNumDescs;
//uint32_t physts = 0;
int32_t rxBytes, i, txNextIndex;


void 	Conf_SSP_as_SPI();
void    Local_ENET_Init();


int32_t incIndex(int32_t index, int32_t max);

/* Initialize MAC descriptors for simple packet receive/transmit */
void InitDescriptors(
	ENET_ENHTXDESC_T *pTXDescs, int32_t numTXDescs,
	ENET_ENHRXDESC_T *pRXDescs, int32_t numRXDescs);

/* Attach a buffer to a descriptor and queue it for reception */
void ENET_RXQueue(void *buffer, int32_t bytes);


/* Returns a pointer to a filled ethernet buffer or NULL if none are available */
void *ENET_RXGet(int32_t *bytes);


/* Attaches a buffer to a transmit descriptor and queues it for transmit */
void ENET_TXQueue(void *buffer, int32_t bytes);


/* Returns a pointer to a buffer that has been transmitted */
void *ENET_TXBuffClaim(void);

//void ENET_SendFrameBlocking1(uint8_t *address, uint8_t *buf,  uint16_t size);
void ENET_SendFrameBlocking(uint8_t *buf,  uint16_t size);
void ENET_SendFrameNonBlocking(uint8_t *buf,  uint16_t size);

uint8_t ReadSwitchRegister( uint8_t reg);

void WriteSwitchRegister( uint8_t reg, uint8_t data);

void ProgramFPGAFirmware();
#endif /* INC_ETHERNET_H_ */
