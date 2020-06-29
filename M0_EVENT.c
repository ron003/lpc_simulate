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
	evstotransmit=evbuf->i_written - evbuf->i_read;

	TRACE(13,"evbuf=%p tot_put=%u evstotransmit=%u", (void*)evbuf, evbuf->numevts, evstotransmit);
	while(evstransmitted<evstotransmit) // Transmit maximum full buffer, no risk of read-write race
	{
		DEBUGOUT("Sending FEB_OK_CDR with data payload.. evbuf->i_written=%u\n", evbuf->i_written);
		Init_FEBDTP_pkt(&pkt1, macaddr,dstmacaddr);
		evsinpack=0;
		paklen=MAXPACKLEN-MAXPAYLOAD;
		dptr=0;
		evbuf->overwritten=0; //reset lost counter
		//fill packet with events
		if ((evbuf->i_read % 1000) == 0)
			TRACE(2, "evbuf->i_read=%u", evbuf->i_read);
		while(evsinpack < maxevsinpack && evstransmitted<evstotransmit)
		{
			uint16_t *adcp, uu;
			Event_t  *evp;
			uint16_t buf_read_idx = evbuf->i_read & (EVBUFSIZE-1);
			memcpy(&(pkt1.Data[dptr]), &(evbuf->buf[buf_read_idx].flags), EVSIZE);
			evp=&evbuf->buf[buf_read_idx];
			TRACE(6,"evbuf->buf[buf_read_idx=%u] ==> flags=0x%08x T0=0x%08x T1=0x%08x coinc=0x%08x",
			      buf_read_idx, evp->flags, evp->T0, evp->T1, evp->coinc);
			adcp=evbuf->buf[buf_read_idx].adc;
#           define ADC(x) (uint64_t)(adcp[x])
			TRACE(7, "x%016lx %016lx %016lx %016lx %016lx %016lx %016lx %016lx",
			      (ADC(31)<<48)|(ADC(30)<<32)|(ADC(29)<<16)|(ADC(28)<<0),
			      (ADC(27)<<48)|(ADC(26)<<32)|(ADC(25)<<16)|(ADC(24)<<0),
			      (ADC(23)<<48)|(ADC(22)<<32)|(ADC(21)<<16)|(ADC(20)<<0),
			      (ADC(19)<<48)|(ADC(18)<<32)|(ADC(17)<<16)|(ADC(16)<<0),
			      (ADC(15)<<48)|(ADC(14)<<32)|(ADC(13)<<16)|(ADC(12)<<0),
			      (ADC(11)<<48)|(ADC(10)<<32)|(ADC( 9)<<16)|(ADC( 8)<<0),
			      (ADC( 7)<<48)|(ADC( 6)<<32)|(ADC( 5)<<16)|(ADC( 4)<<0),
			      (ADC( 3)<<48)|(ADC( 2)<<32)|(ADC( 1)<<16)|(ADC( 0)<<0));
			for (uu=0; uu<31; ++uu)
				if (adcp[uu] != (buf_read_idx*32+uu)) {
					TRACE(0, "i_read=%u unexpected adc value adcp[%u]=0x%04x != (%u*32+%u)",
					      evbuf->i_read, uu, adcp[uu], buf_read_idx, uu);
					exit(1);
				}
			evbuf->i_read++; //increment pointer in loop -- ROLLOVER ACCOUNTED FOR
			evsinpack++;
			paklen += EVSIZE;
			dptr += EVSIZE;
			TRACE(9, "evbuf->numevts=%u ->i_read=%u (after inc)", evbuf->numevts, evbuf->i_read);
			evstransmitted++;
		}
		pkt1.CMD=FEB_DATA_CDR;
		pkt1.REG=evstotransmit-evstransmitted;    //evbuf.numevts; //remaining events
		ENET_SendFrameNonBlocking((uint8_t*)(&pkt1), paklen);
		Board_LED_Toggle(0);
	}
	//evbuf->overwritten=0; //reset lost counter   ??????
	return retval;
} /* TransmitEventBuffer */

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


