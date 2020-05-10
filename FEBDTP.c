/*
 * FEBDTP.c
 *
 *  Created on: Apr 25, 2015
 *      Author: I. Kreslo
 */

#include "FEBDTP.h"

void Init_FEBDTP_pkt(FEBDTP_PKT *pkt, uint8_t* src, uint8_t* dst)
{
	  memcpy(&(pkt->src_mac),src,6);
	  memcpy(&(pkt->dst_mac),dst,6);
//	  pkt->IG=0x00;
//	  pkt->CR=0x01;
//	  pkt->CF=0x0203;
	  pkt->iptype=0x0108; // IP type 0801

}

