/*
 * CITIROC.h
 *
 *  Created on: Apr 27, 2015
 *      Author: lhep
 */

#ifndef INC_CITIROC_H_
#define INC_CITIROC_H_

#define CITIROC_SELECT_SCR 1   //SELECT bit for Slow Control register
#define CITIROC_SELECT_PMR 0   //SELECT bit for Probe MUX register
#define CITIROC_SCR_LEN 1144/8    // length of the slow control register
#define CITIROC_PMR_LEN 224/8  // length of the probe MUX register
#define CITIROC_DR_LEN 32/8    // length of the data MUX register

uint8_t Citiroc_SCR_buf[CITIROC_SCR_LEN];          //buffer for SCR data
uint8_t Citiroc_PMR_buf[CITIROC_PMR_LEN];    //buffer for Probe MUX data

void CITIROC_WriteSCR();
void CITIROC_WritePMR();

#endif /* INC_CITIROC_H_ */
