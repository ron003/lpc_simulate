/*
 * EVENT.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lhep
 */
#include "board.h"
#include <stdio.h>
#include <string.h>
#include <cr_section_macros.h>

#ifndef INC_EVENT_H_
#define INC_EVENT_H_

#define EVBUFSIZE 1024 // Max number of events in evbuf events buffer
#define EVSIZE 80 //size of one event:

typedef struct {
	uint32_t flags; //flags defining event type, 1=T0 reset, 2=T1 reset or 4=scintillator trigger
	uint32_t T0;
	uint32_t T1;
	uint16_t adc[32]; //adc data on 32 channels
	uint32_t coinc;
} Event_t;

typedef struct {
	uint32_t overwritten; // number of overwritten (lost) events          -- incremented by writer
	uint16_t i_written;   //                                              -- incremented by writer
	uint16_t i_read;      // "red"                                        -- incremented by reader
	uint32_t numevts;     // num put into buf (total=numevts+overwritten) -- incremented by writer
	Event_t buf[EVBUFSIZE];
} Evbuf_t;

extern Evbuf_t evbuf;

void ProcessEvent();
bool TransmitEventBuffer(Evbuf_t *evbuf);
bool TransmitFirmwareBlock(addr_t start_addr, uint16_t nblocks);
uint8_t getCRC(uint8_t message[], uint32_t length);

#endif /* INC_EVENT_H_ */
