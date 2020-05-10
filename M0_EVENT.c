/*
 * EVENT.c
 *
 *  Created on: Apr 29, 2015
 *      Author: lhep
 */

#include "EVENT.h"
#include "ethernet.h"
#include <cr_section_macros.h>
#include "FEBDTP.h"



bool TransmitEventBuffer(Evbuf_t *evbuf)
{
	bool retval=true;
	uint16_t paklen=MAXPACKLEN-MAXPAYLOAD; //size of header
	uint16_t evsinpack=0; //events in eth packet
	uint16_t maxevsinpack=(MAXPAYLOAD/EVSIZE);
	uint16_t dptr=0; //pointer to next free position in pkt1.Data
	uint16_t evstransmitted=0;
	uint16_t evstotransmit=0;
	FEBDTP_PKT pkt1;
	evstotransmit=evbuf->numevts;

	TRACE(3,"evbuf=%p evbuf->numevts=%u", (void*)evbuf, evbuf? evbuf->numevts: 0);
	while(evstransmitted<evstotransmit) // Transmit maximum full buffer, no risk of read-write race
	{
		DEBUGOUT("Sending FEB_OK_CDR with data payload.. \n");
		Init_FEBDTP_pkt(&pkt1, macaddr,dstmacaddr);
		evsinpack=0;
		paklen=MAXPACKLEN-MAXPAYLOAD;
		dptr=0;
		evbuf->overwritten=0; //reset lost counter
		//fill packet with events
		while(evsinpack < maxevsinpack && evstransmitted<evstotransmit)
		{
			memcpy(&(pkt1.Data[dptr]), &(evbuf->buf[evbuf->i_first].flags), EVSIZE);
			evbuf->i_first++; if(evbuf->i_first==EVBUFSIZE) evbuf->i_first=0; //increment pointer in loop
			evsinpack++;
			paklen += EVSIZE;
			dptr += EVSIZE;
			evbuf->numevts--;
			evstransmitted++;
		}
		pkt1.CMD=FEB_DATA_CDR;
		pkt1.REG=evstotransmit-evstransmitted;    //evbuf.numevts; //remaining events
		ENET_SendFrameNonBlocking((uint8_t*)(&pkt1), paklen);
		Board_LED_Toggle(0);
	}
	evbuf->overwritten=0; //reset lost counter
	return retval;
}

__RAMFUNC(RAM) uint8_t getCRC(uint8_t message[], uint32_t length)
{
	uint32_t i, j;
	uint8_t crc = 0;

	for (i = 0; i < length; i++)
	{
		crc ^= message[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 1)
				crc ^=  0x91; //CRC7_POLY
			crc >>= 1;
		}
	}
	return crc;
}


bool TransmitFirmwareBlock(addr_t start_addr, uint16_t nblocks)
{
	bool retval=1;
    uint16_t blocks=0;
    uint8_t * ptr;
    FEBDTP_PKT pkt1;
	Init_FEBDTP_pkt(&pkt1, macaddr,dstmacaddr);
	pkt1.CMD=FEB_DATA_FW;
    for(blocks=0; blocks<nblocks; blocks++)
    {
    	ptr=(uint8_t *)(start_addr+blocks*1024);
    	memcpy(pkt1.Data,ptr,1024);
    	pkt1.REG = getCRC(ptr,1024);
    	ENET_SendFrameBlocking((uint8_t*)(&pkt1), MAXPACKLEN-MAXPAYLOAD+1024);
    }
	return retval;
}


