/*
 ===============================================================================
 Name        : FEB_test4_M0.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
 ===============================================================================
 */
#include "board.h"
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>			/* htons */
#include "ethernet.h"
#include "FEBDTP.h"
#include "CITIROC.h"
#include "EVENT.h"
#include "flash.h"

#define TRACE_NAME trace_path_components(__FILE__,1)

#define VERSTR "FEB_rev3_IAP7.013"

static uint8_t*m0m4shmem;
#define M0M4SHMEM m0m4shmem     //0x10088000
#define M0M4STOP (M0M4SHMEM+12) //flags for iter-CPU communications: to stop M4 in RAM code for safe FLASH programming
#define M4M0STOPED (M0M4SHMEM+16) //flags for iter-CPU communications: tells to M0 that M4 is in safe RAM code loop
#define M4M0CFGFPGA (M0M4SHMEM+20) //HACK ABBA

#include <cr_section_macros.h>
#define EMAC_INPUT           (MD_PLN | MD_EZI)
#define EMAC_OUTPUT          (MD_PLN)

 PINMUX_GRP_T spifipinmuxing[] = {
	{0x3, 3,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},	/* SPIFI CLK */
	{0x3, 4,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},	/* SPIFI D3 */
	{0x3, 5,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},	/* SPIFI D2 */
	{0x3, 6,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},	/* SPIFI D1 */
	{0x3, 7,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},	/* SPIFI D0 */
	{0x3, 8,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)}	/* SPIFI CS/SSEL */
};

bool ReceiveProgramFirmwareBlocks(addr_t start_addr, uint16_t nblocks);
void CopyFlashBlocksAndRestart(addr_t start_addr, uint16_t nblocks);
/**
 * Wait for message from M4 core
 */
void MX_CORE_IRQHandler(void) {
	Chip_CREG_ClearM4Event();
	Board_LED_Toggle(0);

}
char VersionString[32];

FEBDTP_PKT pkt1;     // FEBDTP.h:77:}} FEBDTP_PKT; // packet total length 1500 bytes

uint32_t regn = 0;
//	*((uint32_t*)M0M4SHMEM)=&evbuf
Evbuf_t* evbufp;
float* EvRate;
int* DAQ_Enabled;
uint32_t CFGFPGA; //HACKABBA




__RAMFUNC(RAM) void ResetCPU(){
	LPC_WWDT->MOD       = 0;
	LPC_WWDT->TC = 1;
	LPC_WWDT->MOD |= WWDT_WDMOD_WDRESET;
	LPC_WWDT->MOD |=  WWDT_WDMOD_WDEN;
	LPC_WWDT->FEED = 0xAA;
	LPC_WWDT->FEED = 0x55;


return;
}

__RAMFUNC(RAM) void *M0_main(void *arg) {
	addr_t start_addr=0;
	uint16_t nblocks;
	M0M4SHMEM = (uint8_t*)arg;  // reverse engineering the shared memory layout...
	// What is at the beginning of the memory region is a ptr to the evbuf
	
	sprintf(VersionString,"%s",VERSTR);
	evbufp = (Evbuf_t*) (*((Evbuf_t**) M0M4SHMEM)); // main event buffer pointer, see EVENT.h
	DEBUGOUT("M0M4SHMEM=%p evbufp=%p",(void*)M0M4SHMEM, (void*)evbufp);
	EvRate = (float*) (*((float**) (M0M4SHMEM + sizeof(addr_t)))); // event rate gauge pointer, see EVENT.h
	DAQ_Enabled = (int*) (*((int**) (M0M4SHMEM + 2*sizeof(addr_t)))); // DAQ_Enabled flag pointer, see EVENT.h

	*((addr_t*) (M4M0CFGFPGA)) = (addr_t)(&CFGFPGA); // store address of the DAQ Enable flag for M0 core
	CFGFPGA=0x00;

	//configio= (uint32_t*) (*((uint32_t**) (M0M4SHMEM + 24))); // Shared IO Configuration
	//*configio=0x11;
	// Read clock settings and update SystemCoreClock variable
	SystemCoreClockUpdate();
	Board_LED_Toggle(0);
	NVIC_EnableIRQ(M4_IRQn);

	/* SSP initialization for SWITCH control and config*/
	Board_SSP_Init(LPC_SSP);
	Chip_SSP_Init(LPC_SSP);
	Conf_SSP_as_SPI();



//    DEBUGOUT("Starting Ethernet.. \n");

	/* Setup ethernet and PHY */
	Chip_ENET_Init(LPC_ETHERNET);
	Board_ENET_Init();
	Local_ENET_Init();

//	DEBUGOUT("Starting switch.. \n");
	WriteSwitchRegister(1, 1); //start the switch

	TRACE(3,"main - before ProgramFPGAFirmware()");
	ProgramFPGAFirmware();


	TRACE(3,"main - before while(1)");
//	while (1) {}
	while (1) {

		/* Check for receive packets */
		workbuff = ENET_RXGet(&rxBytes);
		if (workbuff) {

			//Action on the received packet
			memcpy((uint8_t*) (&pkt1), workbuff, rxBytes);
			DEBUGOUT("CMD: %04x", pkt1.CMD);
			/* Re-queue the (same) packet again */
			ENET_RXQueue(workbuff, EMAC_ETH_MAX_FLEN);

			if (pkt1.dst_mac[0] == macaddr[0] && pkt1.dst_mac[1] == macaddr[1]
					&& pkt1.dst_mac[2] == macaddr[2]
					&& pkt1.dst_mac[3] == macaddr[3]
					&& pkt1.dst_mac[4] == macaddr[4]
					&& (pkt1.dst_mac[5] == macaddr[5] || pkt1.dst_mac[5]==0xff))
				switch (pkt1.CMD) {
// SWITCH programming commands
				case FEB_GEN_HVON:
					// DEBUGOUT("\nFEB_GEN_HVON received\n");
					// DEBUGOUT("Sending FEB_OK.. \n");
					Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0x1, 12, 1); //PWR_ON
					Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
					pkt1.CMD = FEB_OK;
					pkt1.REG = 0;
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
					Board_LED_Toggle(0);
					break;
				case FEB_GEN_HVOF:
					// DEBUGOUT("\nFEB_GEN_HVON received\n");
					// DEBUGOUT("Sending FEB_OK.. \n");
					Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0x1, 12, 0); //PWR_ON
					Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
					pkt1.CMD = FEB_OK;
					pkt1.REG = 0;
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
					Board_LED_Toggle(0);
					break;
				case FEB_GEN_INIT:
					// DEBUGOUT("\nFEB_GEN_INIT received\n");
					// LPC_RGU->RESET_CTRL0=0x4; //MASTER RESET
					if(pkt1.REG==00) *DAQ_Enabled=0; //Disable DAQ command
					if(pkt1.REG==02) *DAQ_Enabled=1; //Enable DAQ command
					if(pkt1.REG==01) { //Reset buffer command
						evbufp->numevts=0;
						evbufp->i_first=0;
						evbufp->i_last=0xffff; //(equal to -1)
						evbufp->overwritten = 0; // number of overwritten (lost) events
					}
					if(pkt1.REG==0xFF) ResetCPU(); //identic to pressing reset button on the FEB
					// DEBUGOUT("Sending FEB_OK.. \n");
					Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
					pkt1.CMD = FEB_OK;
					pkt1.REG = 0;
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
					Board_LED_Toggle(0);
					break;

				case FEB_SET_FPGA_CFG:
					CFGFPGA = pkt1.REG;
					Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
					pkt1.CMD = FEB_OK;
					pkt1.REG = 0;
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
					Board_LED_Toggle(0);
					break;

				case FEB_SET_RECV:
					// DEBUGOUT("\nFEB_SER_RECV received\n");
					memcpy(dstmacaddr, pkt1.Data, 6);
					if(pkt1.REG != 0) Chip_DAC_UpdateValue(LPC_DAC, pkt1.REG ); //10 ADC~=180ns
					// DEBUGOUT("Sending FEB_OK.. \n");
					Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
					pkt1.CMD = FEB_OK;
					pkt1.REG = 0;
					memcpy(pkt1.Data, VersionString, sizeof(VersionString));
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
					Board_LED_Toggle(0);
					break;
				case FEB_GET_RATE:
					// DEBUGOUT("\nFEB_GET_RATE received\n");
					// DEBUGOUT("Sending FEB_OK.. \n");
					Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
					pkt1.CMD = FEB_OK;
					pkt1.REG = 0;
					memcpy(pkt1.Data, EvRate, sizeof(*EvRate));
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
					Board_LED_Toggle(0);
					break;
				case FEB_RD_SR:
					regn = pkt1.REG;
					// DEBUGOUT("\nFEB_RD_SR received, REG=\n",pkt1.REG);
					// DEBUGOUT("Sending FEB_OK_SR.. \n");
					Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
					pkt1.CMD = FEB_OK_SR;
					pkt1.REG = regn;
					pkt1.Data[0] = ReadSwitchRegister(regn);
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
					Board_LED_Toggle(0);
					break;
				case FEB_WR_SR:
					regn = pkt1.REG;
					WriteSwitchRegister(regn, pkt1.Data[0]);
					// DEBUGOUT("\nFEB_WR_SR received, REG=%02x, Value=%02x\n",pkt1.REG,pkt1.Data[0]);
					// DEBUGOUT("Sending FEB_OK_SR.. \n");
					Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
					pkt1.CMD = FEB_OK_SR;
					pkt1.REG = regn;
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
					Board_LED_Toggle(0);
					break;
				case FEB_RD_SRFF:
					// DEBUGOUT("\nFEB_RD_SRFF received\n");
					// DEBUGOUT("Sending FEB_OK_SR.. \n");
					Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
					pkt1.CMD = FEB_OK_SR;
					for (regn = 0; regn < 256; regn++)
						pkt1.Data[regn] = ReadSwitchRegister(regn);
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 274);
					Board_LED_Toggle(0);
					break;
				case FEB_WR_SRFF:
					regn = pkt1.REG;
					for (regn = 0; regn < 256; regn++)
						WriteSwitchRegister(regn, pkt1.Data[regn]);
					// DEBUGOUT("\nFEB_WR_SRFF received\n",);
					// DEBUGOUT("Sending FEB_OK_SR.. \n");
					Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
					pkt1.CMD = FEB_OK_SR;
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
					Board_LED_Toggle(0);
					break;
// CITIROC programming
				case FEB_RD_SCR:
					// DEBUGOUT("\nFEB_RD_SCR received\n");
					// DEBUGOUT("Sending FEB_OK_SCR.. \n");
					Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
					pkt1.CMD = FEB_OK_SCR;
					for (regn = 0; regn < CITIROC_SCR_LEN; regn++)
						pkt1.Data[regn] = Citiroc_SCR_buf[regn];
					ENET_SendFrameBlocking((uint8_t*) (&pkt1),
							CITIROC_SCR_LEN + 18);
					Board_LED_Toggle(0);
					break;
				case FEB_WR_SCR:
					regn = pkt1.REG;
					for (regn = 0; regn < CITIROC_SCR_LEN; regn++)
						Citiroc_SCR_buf[regn] = pkt1.Data[regn];
					// DEBUGOUT("\nFEB_WR_SCR received\n",);
					CITIROC_WriteSCR();
					// DEBUGOUT("Sending FEB_OK_SCR.. \n");
					Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
					pkt1.CMD = FEB_OK_SCR;
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
					Board_LED_Toggle(0);
					break;

				case FEB_RD_PMR:
					// DEBUGOUT("\nFEB_RD_PMR received\n");
					// DEBUGOUT("Sending FEB_OK_PMR.. \n");
					Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
					pkt1.CMD = FEB_OK_PMR;
					for (regn = 0; regn < CITIROC_PMR_LEN; regn++)
						pkt1.Data[regn] = Citiroc_PMR_buf[regn];
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
					Board_LED_Toggle(0);
					break;
				case FEB_WR_PMR:
					regn = pkt1.REG;
					for (regn = 0; regn < CITIROC_PMR_LEN; regn++)
						Citiroc_PMR_buf[regn] = pkt1.Data[regn];
					// DEBUGOUT("\nFEB_WR_PMR received\n",);
					CITIROC_WritePMR();
					// DEBUGOUT("Sending FEB_OK_PMR.. \n");
					Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
					pkt1.CMD = FEB_OK_PMR;
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
					Board_LED_Toggle(0);
					break;
				case FEB_RD_CDR: //main buffer transmit request
					DEBUGOUT("FEB_RD_CDR received\n");
					if (TransmitEventBuffer(evbufp)) {
						//               if (1) {
						// DEBUGOUT("Sending FEB_OK_CDR.. \n");
						Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
						pkt1.CMD = FEB_EOF_CDR;
						pkt1.REG = 0; //end of transmission
					} else {
						// DEBUGOUT("Sending FEB_ERR_CDR.. \n");
						Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
						pkt1.CMD = FEB_ERR_CDR;
					}
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
					Board_LED_Toggle(0);
					break;
				case FEB_RD_FW:  //request to transmit firmware buffer to host
					nblocks=pkt1.Data[4]; //number of 1024 kB blocks
					nblocks=(nblocks<<8) | pkt1.Data[3]; //number of 1024 kB blocks
					start_addr=(addr_t)0x14000000; //base address of the firmware SPIFI memory
					start_addr+=((pkt1.Data[2]<<16)|(pkt1.Data[1]<<8)|(pkt1.Data[0]));
					if (TransmitFirmwareBlock(start_addr, nblocks)) {
						Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
						pkt1.CMD = FEB_EOF_FW;
						pkt1.REG = getCRC((uint8_t*)start_addr,nblocks*1024);
					} else {
						Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
						pkt1.CMD = FEB_ERR_FW;
					}
					ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
					Board_LED_Toggle(0);
					break;
				case FEB_WR_FW: // address is given in FLASH memory space, starting from 0x0
					nblocks = pkt1.Data[3]; //number of 1024 kB blocks
					//nblocks=(nblocks<<8) | pkt1.Data[3]; //number of 1024 kB blocks
					if (nblocks > 64)
						nblocks = 64;
					start_addr = (addr_t)0;
					start_addr += ((pkt1.Data[2]<<16)|(pkt1.Data[1]<<8)|(pkt1.Data[0]));
					//start address in the firmware QSPI space (starts with 0x100 )
					if (pkt1.REG == 0x0101)
						CopyFlashBlocksAndRestart(start_addr, nblocks);
					else {
						if (ReceiveProgramFirmwareBlocks(start_addr, nblocks)) {
							Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
							pkt1.CMD = FEB_EOF_FW;
							pkt1.REG = getCRC(
									(uint8_t*) (start_addr + 0x14000000),
									nblocks * 1024);
						} else {
							Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
							pkt1.CMD = FEB_ERR_FW;
						}
						ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64);
						Board_LED_Toggle(0);
					}
					break;

				case FEB_DATA_FW:
				case FEB_EOF_FW:
				case FEB_OK_FW:
					Board_LED_Toggle(0);
					break;
				}

		}
		//	__WFI();
	}
	return 0;
}
LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr1= (LPC_SPIFI_CHIPHW_T *)LPC_SPIFI_BASE;

__RAMFUNC(RAM) bool ReceiveProgramFirmwareBlocks(addr_t start_addr, uint16_t nblocks)  //Receive up to 64 kB of firmware and program it to QSPI FLASH
{
	bool retval = 1;
	uint16_t blocks = 0;
	uint8_t *bufptr=(uint8_t*)evbufp; //reuse evbuf for buffer
	int DAQwasEnabled = *DAQ_Enabled;
	*DAQ_Enabled = 0; //Disable DAQ command
	uint8_t crc7;
    int i;
    addr_t addr;
 //   uint32_t statFlash;
 //   uint32_t statSPIFI;

//    Board_LED_Set(0,0);
//	FEBDTP_PKT pkt1;
	Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
	pkt1.CMD = FEB_OK_FW;
	pkt1.REG = nblocks;
	ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64); //send confirmation of readiness to receive NN blocks
	TRACE(3,"In ReceiveProgramFirmwareBlocks before while blocks < %u", nblocks );
	while (blocks < nblocks) { //loop on up to 64 1kB blocks
		workbuff = ENET_RXGet(&rxBytes);
		if (workbuff) {
			memcpy((uint8_t*) (&pkt1), workbuff, 1024+18);
			ENET_RXQueue(workbuff, EMAC_ETH_MAX_FLEN);
			if (pkt1.dst_mac[0] == macaddr[0] && pkt1.dst_mac[1] == macaddr[1]
					&& pkt1.dst_mac[2] == macaddr[2]
					&& pkt1.dst_mac[3] == macaddr[3]
					&& pkt1.dst_mac[4] == macaddr[4]
					&& pkt1.dst_mac[5] == macaddr[5])  //No multicast packages here, only unambiguously addressed!
			{
//			    Board_LED_Set(0,1);
				// copy 1kB Data to buffer, reuse evbuf for buffer
				crc7=pkt1.REG;
				memcpy(bufptr,pkt1.Data,1024); //copy block to buffer
				Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
				pkt1.REG = getCRC(bufptr,1024);
				if(pkt1.REG==crc7) pkt1.CMD = FEB_OK_FW; //check CRC consistency
				else pkt1.CMD = FEB_ERR_FW;
				ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64); //send confirmation of received block
                bufptr+=1024;
                blocks++;
   //             Board_LED_Set(0,0);
			}
		}
	}
	//all blocks are in RAM buffer now (evbuf), time to program SPIFI
	bufptr=(uint8_t*)evbufp;
//	Chip_SCU_SetPinMuxing(spifipinmuxing, sizeof(spifipinmuxing) / sizeof(PINMUX_GRP_T));
	//Chip_Clock_SetDivider(CLK_IDIV_E, CLKIN_MAINPLL, 2);
//	Chip_Clock_SetBaseClock(CLK_BASE_SPIFI, CLKIN_IDIVE, true, false);
    *((uint8_t*)M0M4STOP)=1;
	volatile uint8_t M4STOPPED;
	M4STOPPED=*((uint8_t*)M4M0STOPED);
	while(M4STOPPED==0){M4STOPPED=*((uint8_t*)M4M0STOPED);} //wait for M4 to enter safe RAM loop and disable its INTs
	__disable_irq();
// FlashOperations();
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
//	FLASH_Reset();
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
//	FLASH_SetStatus(0x200); //Quad enable only
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
	FLASH_SetMemMode(0);
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
    FLASH_GetPartID();
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
	FLASH_UnLock();
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
	int first_4kB_block;
	int last_4kB_block;
	first_4kB_block=(ulong)start_addr/4096;
	last_4kB_block=((ulong)start_addr+nblocks*1024-1)/4096;
	for(i=first_4kB_block; i<=last_4kB_block;i++)
		{
		  FLASH_Erase4kB(i);
//			statFlash=FLASH_GetStatus();
//			statSPIFI=pSpifiCtrlAddr1->STAT;
		}
	addr=start_addr;
	int pages;
    pages=0;
	while(pages<nblocks*4) //256 Bytes pages, four in each 1kB block
	{
		FLASH_PageProg(addr,bufptr,256);
//		statFlash=FLASH_GetStatus();
//		statSPIFI=pSpifiCtrlAddr1->STAT;
		addr+=256;
		bufptr+=256;
		pages++;
	}
//	FLASH_Reset();
//	FLASH_PageRead(addr,bufptr); //read 256 bytes
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
	FLASH_SetMemMode(1);
//	pSpifiCtrlAddr1->STAT=0x2000024;
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
    __enable_irq();
	*((uint8_t*)M0M4STOP)=0; //Release M4 for normal operation
	//Finished with SPIFI programming, reset event buffer
	evbufp->numevts = 0;
	evbufp->i_first = 0;
	evbufp->i_last = 0xffff; //(equal to -1)
	evbufp->overwritten = 0; // number of overwritten (lost) events
	*DAQ_Enabled = DAQwasEnabled; //restart DAQ if it was running
	return retval;
}

__RAMFUNC(RAM) void CopyFlashBlocksAndRestart(addr_t start_addr, uint16_t nblocks)
{
//	   uint32_t statFlash;
//	    uint32_t statSPIFI;
	//Chip_Clock_SetDivider(CLK_IDIV_E, CLKIN_MAINPLL, 2);
    addr_t srcaddr,dstaddr;
	uint8_t *bufptr=(uint8_t*)evbufp; //reuse evbuf for buffer
	*((uint8_t*)M0M4STOP)=1;
	volatile uint8_t M4STOPPED;
	M4STOPPED=*((uint8_t*)M4M0STOPED);
	while(M4STOPPED==0){M4STOPPED=*((uint8_t*)M4M0STOPED);} //wait for M4 to enter safe RAM loop and disable its INTs
	__disable_irq();
//	// FlashOperations();
//		statFlash=FLASH_GetStatus();
//		statSPIFI=pSpifiCtrlAddr1->STAT;
//	//	FLASH_Reset();
//		statFlash=FLASH_GetStatus();
//		statSPIFI=pSpifiCtrlAddr1->STAT;
//	//	FLASH_SetStatus(0x200); //Quad enable only
//		statFlash=FLASH_GetStatus();
//		statSPIFI=pSpifiCtrlAddr1->STAT;
		FLASH_SetMemMode(0);
//		statFlash=FLASH_GetStatus();
//		statSPIFI=pSpifiCtrlAddr1->STAT;
	    FLASH_GetPartID();
//		statFlash=FLASH_GetStatus();
//		statSPIFI=pSpifiCtrlAddr1->STAT;
		FLASH_UnLock();
//		statFlash=FLASH_GetStatus();
//		statSPIFI=pSpifiCtrlAddr1->STAT;
	srcaddr=start_addr;
	dstaddr=0;
	int last_4kB_block;
	last_4kB_block=(nblocks*1024-1)/4096;
	for(i=0; i<=last_4kB_block;i++) {
		FLASH_Erase4kB(i);
//		statFlash=FLASH_GetStatus();
//		statSPIFI=pSpifiCtrlAddr1->STAT;
	}

	int pages;
    pages=0;
    while(pages<nblocks*4) //256 Bytes pages, four in each 1kB block
	{
		FLASH_PageRead(srcaddr,bufptr); //read 256 bytes
//		statFlash=FLASH_GetStatus();
//		statSPIFI=pSpifiCtrlAddr1->STAT;
		FLASH_PageProg(dstaddr,bufptr,256);
//		statFlash=FLASH_GetStatus();
//		statSPIFI=pSpifiCtrlAddr1->STAT;
		srcaddr+=256;
		dstaddr+=256;
		pages++;
	}
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
	FLASH_SetMemMode(1);
 	//Finished with SPIFI programming, reset event buffer

	ResetCPU();
	while(1){} //just in case we reach this point
}

/*
void SendFPGAProgWord(	uint32_t data)
{

	uint32_t ib;
	LPC_GPIO_PORT->B[5][13] = true; //	TS_CLK - latch data bus
	LPC_GPIO_PORT->B[7][23] = false; // DSPI_CLK cycle clock
	LPC_GPIO_PORT->B[7][23] = true; // DSPI_CLK cycle clock
	LPC_GPIO_PORT->B[7][23] = true; // DSPI_CLK cycle clock
	LPC_GPIO_PORT->B[7][23] = false; // DSPI_CLK cycle clock
	LPC_GPIO_PORT->B[5][13] = false; //	TS_CLK - latch data bus
	LPC_GPIO_PORT->B[5][13] = false; //	TS_CLK - latch data bus
				// here data should be loaded to serialiser, now read it out
	LPC_GPIO_PORT->B[1][11] = true; // DRDY out to enable shift register
	LPC_GPIO_PORT->B[1][11] = true; // DRDY out to enable shift register
	LPC_GPIO_PORT->B[7][23] = true; // DSPI_CLK cycle clock
	LPC_GPIO_PORT->B[7][23] = true; // DSPI_CLK cycle clock
	LPC_GPIO_PORT->B[7][23] = false; // DSPI_CLK cycle clock

	LPC_GPIO_PORT->B[6][20] = (data>>0)&0x1; // MOSI


	for(ib=0; ib<32; ib++)
	{
		mask=(mask >> 1);
		LPC_GPIO_PORT->B[7][23] = true; // DSPI_CLK cycle clock
		LPC_GPIO_PORT->B[7][23] = false; // DSPI_CLK cycle clock
		LPC_GPIO_PORT->B[6][20] = (data>>(ib+1))&0x1; // MOSI
	}
	LPC_GPIO_PORT->B[6][20] = 0;


	LPC_GPIO_PORT->B[1][11] = false; // DRDY out to disable shift register

}*/

/*
__RAMFUNC(RAM) bool ReceiveFPGAProgramFirmwareBlocks(uint32_t start_addr, uint16_t nblocks)  //Receive up to 64 kB of firmware and program it to QSPI FLASH
{
	bool retval = 1;
	uint16_t blocks = 0;
	uint8_t *bufptr=(uint8_t*)evbufp; //reuse evbuf for buffer
	int DAQwasEnabled = *DAQ_Enabled;
	*DAQ_Enabled = 0; //Disable DAQ command
	uint8_t crc7;
    int i;
    uint32_t addr;
 //   uint32_t statFlash;
 //   uint32_t statSPIFI;

//    Board_LED_Set(0,0);
//	FEBDTP_PKT pkt1;
	Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
	pkt1.CMD = FEB_OK_FW;
	pkt1.REG = nblocks;
	ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64); //send confirmation of readiness to receive NN blocks
	while (blocks < nblocks) { //loop on up to 64 1kB blocks
		workbuff = ENET_RXGet(&rxBytes);
		if (workbuff) {
			memcpy((uint8_t*) (&pkt1), workbuff, 1024+18);
			ENET_RXQueue(workbuff, EMAC_ETH_MAX_FLEN);
			if (pkt1.dst_mac[0] == macaddr[0] && pkt1.dst_mac[1] == macaddr[1]
					&& pkt1.dst_mac[2] == macaddr[2]
					&& pkt1.dst_mac[3] == macaddr[3]
					&& pkt1.dst_mac[4] == macaddr[4]
					&& pkt1.dst_mac[5] == macaddr[5])  //No multicast packages here, only unambiguously addressed!
			{
//			    Board_LED_Set(0,1);
				// copy 1kB Data to buffer, reuse evbuf for buffer
				crc7=pkt1.REG;
				memcpy(bufptr,pkt1.Data,1024); //copy block to buffer
				Init_FEBDTP_pkt(&pkt1, macaddr, dstmacaddr);
				pkt1.REG = getCRC(bufptr,1024);
				if(pkt1.REG==crc7) pkt1.CMD = FEB_OK_FW; //check CRC consistency
				else pkt1.CMD = FEB_ERR_FW;
				ENET_SendFrameBlocking((uint8_t*) (&pkt1), 64); //send confirmation of received block
                bufptr+=1024;
                blocks++;
   //             Board_LED_Set(0,0);
			}
		}
	}
	//all blocks are in RAM buffer now (evbufp), time to program SPIFI
	bufptr=(uint8_t*)evbufp;





//	Chip_SCU_SetPinMuxing(spifipinmuxing, sizeof(spifipinmuxing) / sizeof(PINMUX_GRP_T));
	//Chip_Clock_SetDivider(CLK_IDIV_E, CLKIN_MAINPLL, 2);
//	Chip_Clock_SetBaseClock(CLK_BASE_SPIFI, CLKIN_IDIVE, true, false);
    *((uint8_t*)M0M4STOP)=1;
	volatile uint8_t M4STOPPED;
	M4STOPPED=*((uint8_t*)M4M0STOPED);
	while(M4STOPPED==0){M4STOPPED=*((uint8_t*)M4M0STOPED);} //wait for M4 to enter safe RAM loop and disable its INTs
	__disable_irq();
// FlashOperations();
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
//	FLASH_Reset();
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
//	FLASH_SetStatus(0x200); //Quad enable only
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
	FLASH_SetMemMode(0);
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
    FLASH_GetPartID();
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
	FLASH_UnLock();
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
	int first_4kB_block;
	int last_4kB_block;
	first_4kB_block=start_addr/4096;
	last_4kB_block=(start_addr+nblocks*1024-1)/4096;
	for(i=first_4kB_block; i<=last_4kB_block;i++)
		{
		  FLASH_Erase4kB(i);
//			statFlash=FLASH_GetStatus();
//			statSPIFI=pSpifiCtrlAddr1->STAT;
		}
	addr=start_addr;
	int pages;
    pages=0;
	while(pages<nblocks*4) //256 Bytes pages, four in each 1kB block
	{
		FLASH_PageProg(addr,bufptr,256);
//		statFlash=FLASH_GetStatus();
//		statSPIFI=pSpifiCtrlAddr1->STAT;
		addr+=256;
		bufptr+=256;
		pages++;
	}
//	FLASH_Reset();
//	FLASH_PageRead(addr,bufptr); //read 256 bytes
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
	FLASH_SetMemMode(1);
//	pSpifiCtrlAddr1->STAT=0x2000024;
//	statFlash=FLASH_GetStatus();
//	statSPIFI=pSpifiCtrlAddr1->STAT;
    __enable_irq();
	*((uint8_t*)M0M4STOP)=0; //Release M4 for normal operation
	//Finished with SPIFI programming, reset event buffer
	evbufp->numevts = 0;
	evbufp->i_first = 0;
	evbufp->i_last = 0xffff; //(equal to -1)
	evbufp->overwritten = 0; // number of overwritten (lost) events
	*DAQ_Enabled = DAQwasEnabled; //restart DAQ if it was running
	return retval;
}
*/


void      Chip_ENET_Init(LPC_ENET_T *pENET)
{TRACE(3,"Chip_ENET_Init(%p)",(void*)pENET);}

void      Chip_SSP_Init(LPC_SSP_T  *pSSP)
{TRACE(3,"Chip_SSP_Init(%p)",(void*)pSSP);}

void      CITIROC_WritePMR()
{TRACE(3,"CITIROC_WritePMR()");}

void      CITIROC_WriteSCR()
{TRACE(3,"CITIROC_WriteSCR()");}

void      Conf_SSP_as_SPI()
{TRACE(3,"Conf_SSP_as_SPI()");}

// This returns NULL if no data or a ptr to a global data buf and the number of bytes is
// returned via ptr to bytes
static FEBDTP_PKT glb_pkt_buf={.dst_mac={1,2,3,4,5,6},.src_mac={1,2,3,4,5,6}};
void* ENET_RXGet(int32_t *bytes)
{	TRACE(3,"ENET_RXGet(%p)",(void*)bytes); *bytes=sizeof(FEBDTP_PKT);
	glb_pkt_buf.iptype=htons(0x0801);
	glb_pkt_buf.CMD=FEB_RD_CDR;
	return &glb_pkt_buf;
}

void      ENET_RXQueue(void *buffer, int32_t bytes)
{TRACE(3,"ENET_RXQueue(%p,%d)",buffer,bytes);}

bool      FLASH_Erase4kB(int blk)
{TRACE(3,"FLASH_Erase4kB(%d)",blk);}

uint32_t  FLASH_GetPartID()
{TRACE(3,"FLASH_GetPartID"); return 0;}

bool      FLASH_PageProg(addr_t addr, uint8_t *buf, uint32_t bytes)
{TRACE(3,"FLASH_PageProg(%p,%p,%u)",(void*)addr,(void*)buf,bytes); return true;}

bool      FLASH_PageRead(addr_t addr, uint8_t *buf)
{TRACE(3,"FLASH_PageRead(%p,%p)",(void*)addr,(void*)buf); return true;}

void      FLASH_SetMemMode(bool enMMode)
{TRACE(3,"FLASH_SetMemMode(%d)",enMMode);}

bool      FLASH_UnLock()
{TRACE(3,"FLASH_UnLock()"); return true;}

void      Local_ENET_Init()
{uint8_t x[]={1,2,3,4,5,6};TRACE(3,"Local_ENET_Init -- seting macaddr to 01-02-03-04-05-06"); memcpy(macaddr,x,6);}

void      ProgramFPGAFirmware()
{TRACE(3,"ProgramFPGAFirmware");}

uint8_t   ReadSwitchRegister(uint8_t reg)
{TRACE(3,"ReadSwitchRegister(%u)",reg); return 0;}

void      WriteSwitchRegister(uint8_t reg, uint8_t data)
{TRACE(3,"WriteSwitchRegister(%u,%u)",reg,data);}
