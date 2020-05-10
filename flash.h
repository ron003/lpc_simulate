/*
 * flash.h
 *
 *  Created on: Sep 11, 2015
 *      Author: lhep
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include <stdint.h>
#include "board.h"
#include <cr_section_macros.h>

//			"W25Q80BV",
//			{{0xEF, 0x40, 0x14}, 0, {0}},	/* JEDEC ID, extCount, ext data  */
//			(SPIFI_CAP_DUAL_READ | SPIFI_CAP_QUAD_READ | SPIFI_CAP_QUAD_WRITE | SPIFI_CAP_FULLLOCK |
//			 SPIFI_CAP_NOBLOCK | SPIFI_CAP_SUBBLKERASE),
#define NUMBLOCKS			16						/* # of blocks */
#define BLOCK_SIZE			0x10000				/* block size */
#define NUMSUBBLOCKS			256						/* # of blocks */
#define SUBBLOCK_SIZE			0x1000				/* block size */
//			256,					/* # of sub-blocks */
//			0x1000,					/* sub-block size */
#define PAGESIZE		0x100					/* page size */
//			MAX_SINGLE_READ,					/* max single read bytes */
//			104,				/* max clock rate in Mhz */
//			104,				/* max read clock rate in MHz */
//			104,				/* max high speed read clock rate in MHz */
//			104,				/* max program clock rate in MHz */
//			104,				/* max high speed program clock rate in MHz */
//			FX_spifiDeviceDataInitDeinit,	/* (Fx Id) use generic deviceInit / deInit */
//			FX_spifiDeviceDataClearStatusNone,	/* (Fx Id) Does not have persistent status */
//			FX_spifiDeviceDataGetStatusW25Q80BV,	/* (Fx Id) getStatus */
//			FX_spifiDeviceDataSetStatusS25FL032P,	/* (Fx Id) setStatus (uses S25FL032P variant) */
//			FX_spifiDeviceDataSetOptsQuadModeBit9,		/* (Fx Id) to set/clr options */
//			FX_spifiDeviceInitReadCommand,	/* (Fx Id) to get memoryMode Cmd */
//			FX_spifiDeviceInitWriteCommand	/* (Fx Id) to get program Cmd */



/* Command definitions. Only used commands are defined. */
#define CMD_0B_FAST_READ            0x0B		/**< Read Data bytes at Fast Speed */
#define CMD_BB_DIOR                 0xBB		/**< Dual IORead (all dual except op code( */
#define CMD_EB_QIOR                 0xEB		/**< Quad IORead (all quad except op code) */
#define CMD_0C_FAST_READ            0x0C		/**< Read Data (4B addr) bytes at Fast Speed */
#define CMD_BC_DIOR                 0xBC		/**< Dual IORead (4B addr) (all dual except op code( */
#define CMD_EC_QIOR                 0xEC		/**< Quad IORead (4B addr) (all quad except op code) */
#define CMD_06_WREN                 0x06		/**< Write Enable */
#define CMD_20_P4E                  0x20		/**< 4 KB Parameter Sector Erase */
#define CMD_C7_BE                   0xC7		/**< Bulk Erase */
#define CMD_D8_SE                   0xD8		/**< Sector Erase */
#define CMD_DC_SE					0xDC		/**< Sector erase (4B addr) */
#define CMD_02_PP                   0x02		/**< Page Programming */
#define CMD_12_PP                   0x12		/**< Page Programming (4B addr) */
#define CMD_05_RDSR1                0x05		/**< Read Status Register 1 */
#define CMD_35_RDSR2                0x35		/**< Read Status Register 2 */
#define CMD_33_RDSR3                0x33		/**< Read Status Register 3 */
#define CMD_01_WSR                  0x01		/**< Write Status Registers */
#define CMD_30_CSR                  0x30		/**< Reset the Erase and Program Fail Flag (SR5 and SR6) and restore normal operation) */
#define CMD_32_QPP                  0x32		/**< Quad Page Programming */
#define CMD_34_QPP                  0x34		/**< Quad Page Programming (4B addr) */
#define CMD_38_QPP_MACRONIX         0x38		/**< Quad Page Programming for 25L3235E */


//COMMANDS
#define W_EN 	0x06	//write enable
#define W_DE	0x04	//write disable
#define R_SR1	0x05	//read status reg 1
#define R_SR2	0x35	//read status reg 2
#define W_SR	0x01	//write status reg
#define PAGE_PGM	0x02	//page program
#define QPAGE_PGM	0x32	//quad input page program
#define BLK_E_64K	0xD8	//block erase 64KB
#define BLK_E_32K	0x52	//block erase 32KB
#define SECTOR_E	0x20	//sector erase 4KB
#define CHIP_ERASE	0xc7	//chip erase
#define CHIP_ERASE2	0x60	//=CHIP_ERASE
#define E_SUSPEND	0x75	//erase suspend
#define E_RESUME	0x7a	//erase resume
#define PDWN		0xb9	//power down
#define HIGH_PERF_M	0xa3	//high performance mode
#define CONT_R_RST	0xff	//continuous read mode reset
#define RELEASE		0xab	//release power down or HPM/Dev ID (deprecated)
#define R_MANUF_ID	0x90	//read Manufacturer and Dev ID (deprecated)
#define R_UNIQUE_ID	0x4b	//read unique ID (suggested)
#define R_JEDEC_ID	0x9f	//read JEDEC ID = Manuf+ID (suggested)
#define READ		0x03
#define FAST_READ	0x0b

#define SR1_BUSY_MASK	0x01
#define SR1_WEN_MASK	0x02

#define WINBOND_MANUF	0xef

#define DEFAULT_TIMEOUT 200

/**
 * @brief Possible device capabilities returned from getInfo()
 */
#define SPIFI_CAP_DUAL_READ         (1 << 0)		/**< Supports DUAL read mode */
#define SPIFI_CAP_DUAL_WRITE        (1 << 1)		/**< Supports DUAL write mode */
#define SPIFI_CAP_QUAD_READ         (1 << 2)		/**< Supports QUAD read mode */
#define SPIFI_CAP_QUAD_WRITE        (1 << 3)		/**< Supports QUAD write mode */
#define SPIFI_CAP_FULLLOCK          (1 << 4)		/**< Full device lock supported */
#define SPIFI_CAP_BLOCKLOCK         (1 << 5)		/**< Individual block device lock supported */
#define SPIFI_CAP_SUBBLKERASE       (1 << 6)		/**< Sub-block erase supported */
#define SPIFI_CAP_4BYTE_ADDR		(1 << 7)		/**< Supports 4 Byte addressing */
#define SPIFI_CAP_NOBLOCK           (1 << 16)		/**< Non-blocking mode supported */
/**
 * @brief SPIFI controller control register bit definitions
 */
#define SPIFI_CTRL_TO(t)        ((t) << 0)		/**< SPIFI timeout */
#define SPIFI_CTRL_CSHI(c)      ((c) << 16)		/**< SPIFI chip select minimum high time */
#define SPIFI_CTRL_DATA_PREFETCH_DISABLE(d) ((d) << 21)	/**< SPIFI memMode prefetch enable*/
#define SPIFI_CTRL_INTEN(i)     ((i) << 22)		/**< SPIFI cmdComplete irq enable */
#define SPIFI_CTRL_MODE3(m)     ((m) << 23)		/**< SPIFI mode3 config */
#define SPIFI_CTRL_PREFETCH_DISABLE(d) ((d) << 27)	/**< SPIFI cache prefetch enable */
#define SPIFI_CTRL_DUAL(d)      ((d) << 28)		/**< SPIFI enable dual */
#define SPIFI_CTRL_RFCLK(m)     ((m) << 29)		/**< SPIFI clock edge config */
#define SPIFI_CTRL_FBCLK(m)     ((m) << 30)		/**< SPIFI feedback clock select */
#define SPIFI_CTRL_DMAEN(m)     ((m) << 31)		/**< SPIFI dma enable */

/**
 * @brief SPIFI controller status register bit definitions
 */
#define SPIFI_STAT_RESET        (1 << 4)		/**< SPIFI reset */
#define SPIFI_STAT_INTRQ        (1 << 5)		/**< SPIFI interrupt request */
#define SPIFI_STAT_CMD          (1 << 1)		/**< SPIFI command in progress */
#define SPIFI_STAT_MCINIT               (1)					/**< SPIFI MCINIT */
/**
 * @brief SPIFI controller command register bit definitions
 */
#define SPIFI_CMD_DATALEN(l)    ((l) << 0)		/**< SPIFI bytes to send or receive */
#define SPIFI_CMD_POLLRS(p)     ((p) << 14)		/**< SPIFI enable poll */
#define SPIFI_CMD_DOUT(d)       ((d) << 15)		/**< SPIFI data direction is out */
#define SPIFI_CMD_INTER(i)      ((i) << 16)		/**< SPIFI intermediate bit length */
#define SPIFI_CMD_FIELDFORM(p)  ((p) << 19)		/**< SPIFI 2 bit data/cmd mode control */
#define SPIFI_CMD_FRAMEFORM(f)  ((f) << 21)		/**< SPIFI op and adr field config */
#define SPIFI_CMD_OPCODE(o)     ((uint32_t) (o) << 24)	/**< SPIFI 8-bit command code */

/**
 * @brief frame form definitions
 */
typedef enum {
	SPIFI_FRAMEFORM_OP              = 1,
	SPIFI_FRAMEFORM_OP_1ADDRESS     = 2,
	SPIFI_FRAMEFORM_OP_2ADDRESS     = 3,
	SPIFI_FRAMEFORM_OP_3ADDRESS     = 4,
	SPIFI_FRAMEFORM_OP_4ADDRESS     = 5,
	SPIFI_FRAMEFORM_NOOP_3ADDRESS   = 6,
	SPIFI_FRAMEFORM_NOOP_4ADDRESS   = 7
} SPIFI_FRAMEFORM_T;

/**
 * @brief serial type definitions
 */
typedef enum {
	SPIFI_FIELDFORM_ALL_SERIAL             = 0,
	SPIFI_FIELDFORM_SERIAL_OPCODE_ADDRESS  = 1,
	SPIFI_FIELDFORM_SERIAL_OPCODE          = 2,
	SPIFI_FIELDFORM_NO_SERIAL              = 3
} SPIFI_FIELDFORM_T;


/**
 * @brief	SPIFI controller hardware register structure
 */

typedef struct LPC_SPIFI_CHIPHW {
	volatile    uint32_t CTRL;				/**< SPIFI control register */
	volatile    uint32_t CMD;					/**< SPIFI command register */
	volatile    uint32_t ADDR;				/**< SPIFI address register */
	volatile    uint32_t DATINTM;			/**< SPIFI intermediate data register */
	volatile    uint32_t CACHELIMIT;	/**< SPIFI cache limit register */
	union {
		volatile    uint8_t DAT8;				/**< SPIFI 8 bit data */
		volatile    uint16_t DAT16;			/**< SPIFI 16 bit data */
		volatile    uint32_t DAT32;			/**< SPIFI 32 bit data */
	};

	volatile    uint32_t MEMCMD;			/**< SPIFI memory command register */
	volatile    uint32_t STAT;				/**< SPIFI status register */
} LPC_SPIFI_CHIPHW_T;

/* Common status register definitions */
/* Status Register Write Disable,
   1 = Protects when W# is low,
   0 = No protection, even when W# is low */
#define STATUS_SRWD                   (1 << 7)

/* Block protect bits,
   Protects upper half of address range in 5 sizes */
#define STATUS_BPMASK                 (7 << 2)
/* Write Enable Latch,
   1 = Device accepts Write Status Register, program, or erase commands,
   0 = Ignores Write Status Register, program, or erase commands */
#define STATUS_WEL                    (1 << 1)
/* Write in Progress,
   1 = Device Busy. A Write Status Register, program, or erase,
   0 = Ready. Device is in standby mode and can accept commands. */
#define STATUS_WIP                    (1 << 0)

/* Virtual status bits
   (i.e moved to byte 4 so they don't conflict with bits in lower 3 bytes */
/* Programming Error Occurred,
   0 = No Error,
   1 = Error occurred */
#define STATUS_P_ERR                  (1 << 24)
/* Erase Error Occurred,
   0 = No Error,
   1 = Error occurred */
#define STATUS_W_ERR                  (1 << 25)


uint32_t FLASH_GetPartID();
void FLASH_WaitCMD();
void FLASH_WaitRESET();
void FLASH_Reset();
void FLASH_SetMemMode(bool enMMode);
uint32_t FLASH_GetOpts();
uint32_t FLASH_GetStatus();
void FLASH_SetStatus(uint32_t stat);
//void FLASH_ClearStatus();
bool FLASH_EraseAll();
bool FLASH_Erase64kB(int blk);
bool FLASH_Erase4kB(int blk);
bool FLASH_Lock();
bool FLASH_UnLock();
bool FLASH_PageProg(addr_t addr, uint8_t *buf, uint32_t bytes);
bool FLASH_PageRead(addr_t addr, uint8_t *buf); //reads 256 bytes

#endif /* INC_FLASH_H_ */
