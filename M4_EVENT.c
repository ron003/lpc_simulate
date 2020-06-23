/*
 * EVENT.c
 *
 *  Created on: Apr 29, 2015
 *      Author: lhep
 */

#include "EVENT.h"
//#include "ethernet.h"
#include <cr_section_macros.h>
#include "FEBDTP.h"

#define M0M4SHMEM 0x10088000
#define M4M0CFGFPGA (M0M4SHMEM+20) //HACK ABBA

volatile  uint16_t VCXO_Corr=500;
volatile uint64_t ts0AVE=0;
volatile int ts0IND=0; //number of samples accumulated in averaging var

volatile uint32_t *CFGFPGA = 0;


#define AVMAX0 20  //number of samples to average

__RAMFUNC(RAM2) uint32_t GrayToBin(uint32_t n)
{
	uint32_t res=0;
	int a[32],b[32],i=0;
	for(i=0; i<32; i++){
		if((n & 0x80000000)>0) a[i]=1;
		else a[i]=0;
		n=n<<1;
	}
	b[0]=a[0];
	for(i=1; i<32; i++){
		if(a[i]>0) if(b[i-1]==0) b[i]=1; else b[i]=0;
		else b[i]=b[i-1];
	}
	for(i=0; i<32; i++){
		res=(res<<1);
		res=(res | b[i]);
	}
	return res;
}

__RAMFUNC(RAM2)  void CorrectOscillator(uint32_t ts0)
{
	volatile uint32_t tmp;
//	volatile float Diff;
	volatile int iDiff;
	tmp=(GrayToBin(ts0 >>2))<<2;
    tmp=tmp | (ts0 & 0x00000003);
    ts0AVE=ts0AVE+tmp+5;
    ts0IND++;
    if(ts0IND<AVMAX0) return;
    VCXO_Corr = ((LPC_DAC->CR)>>6) & 0x03FF;
    ts0AVE = ts0AVE/ts0IND;
    iDiff=ts0AVE-(uint64_t)1000000000;
    VCXO_Corr = VCXO_Corr - iDiff/5.2;
	/* Update value */
	tmp = LPC_DAC->CR & DAC_BIAS_EN;
	tmp |= DAC_VALUE(VCXO_Corr);
	LPC_DAC->CR = tmp;
	ts0AVE=0;ts0IND=0;
}

static uint32_t static_count=0;

__RAMFUNC(RAM2) void ProcessEvent() {
	uint8_t ch = 32;
	int ib;
	volatile uint32_t val = 0;
	uint32_t ts0,ts1,mask, coinc_bits;
	uint32_t config_outch = 0;
	uint32_t missed;
	uint16_t ilast = evbuf.i_last;
// Increment all what needed
	ilast++;
	if (ilast == EVBUFSIZE)
		ilast = 0; //increment pointer in loop

	if (evbuf.numevts < EVBUFSIZE)
		evbuf.numevts++;
	else {
		evbuf.overwritten++; // buffer is full, so we're overwriting
		evbuf.i_first++;
		if (evbuf.i_first == EVBUFSIZE)
			evbuf.i_first = 0; //increment pointer in loop
	}
	TRACE(8,"evbuf.i_last=%u .numevts=%u .overwritten=%u EVBUFSIZE(evts)=%d",
	      evbuf.i_last, evbuf.numevts, evbuf.overwritten, EVBUFSIZE);

	//access to shared memory to check value of config_outch
	//config_outch = *((uint32_t*) (M0M4SHMEM + 20));

	//READ TIME STAMP and missed events counter
  ts0=0; ts1=0; missed=0; coinc_bits=0;
			//latch time stamp
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

# if 0
			CFGFPGA = (uint32_t*) (*((uint32_t**) (M4M0CFGFPGA)));

			config_outch=*CFGFPGA;
			LPC_GPIO_PORT->B[6][20] = (config_outch>>0)&0x1; // MOSI

			mask=0x80000000;
			for(ib=0; ib<32; ib++)
			{
				if(LPC_GPIO_PORT->B[4][12]>0) ts0=(ts0 | mask);
				mask=(mask >> 1);
				LPC_GPIO_PORT->B[7][23] = true; // DSPI_CLK cycle clock
//				LPC_GPIO_PORT->B[7][23] = true; // DSPI_CLK cycle clock
				LPC_GPIO_PORT->B[7][23] = false; // DSPI_CLK cycle clock
				LPC_GPIO_PORT->B[6][20] = (config_outch>>(ib+1))&0x1; // MOSI
			}
			LPC_GPIO_PORT->B[6][20] = 0;
# else
			LPC_GPIO_PORT->B[6][20] = config_outch;
			ts0 = static_count++;
# endif
			mask=0x80000000;
			for(ib=0; ib<32; ib++)
			{
				if(LPC_GPIO_PORT->B[4][12]>0) ts1=(ts1 | mask);
				mask=(mask >> 1);
				LPC_GPIO_PORT->B[7][23] = true; // DSPI_CLK cycle clock
//				LPC_GPIO_PORT->B[7][23] = true; // DSPI_CLK cycle clock
				LPC_GPIO_PORT->B[7][23] = false; // DSPI_CLK cycle clock
			}
			mask=0x00800000;
			for(ib=0; ib<8; ib++)
			{
				if(LPC_GPIO_PORT->B[4][12]>0) missed=(missed | mask); // 8 bits after 8 MSB out of 32 of "uint32 missed" contain this 8-bits counter
				mask=(mask >> 1);
				LPC_GPIO_PORT->B[7][23] = true; // DSPI_CLK cycle clock
//				LPC_GPIO_PORT->B[7][23] = true; // DSPI_CLK cycle clock
				LPC_GPIO_PORT->B[7][23] = false; // DSPI_CLK cycle clock
			}

			//ABBA:

			mask=0x00008000;
			for(ib=0; ib<16; ib++)
			{
				if(LPC_GPIO_PORT->B[4][12]>0) coinc_bits=(coinc_bits | mask);
				mask=(mask >> 1);
				LPC_GPIO_PORT->B[7][23] = true; // DSPI_CLK cycle clock
				LPC_GPIO_PORT->B[7][23] = false; // DSPI_CLK cycle clock
			}



			LPC_GPIO_PORT->B[1][11] = false; // DRDY out to disable shift register
	//END READ TIME STAMP


	//Reset CITIROC data read register Active low
	LPC_GPIO_PORT->B[0][7] = false; //GPIO0[7],RESETB_READ
	LPC_GPIO_PORT->B[0][7] = true; //GPIO0[7],RESETB_READ

	// Set CITIROC for data register readout
	LPC_GPIO_PORT->B[5][6] = true; //GPIO5[6],SRIN_READ
	LPC_GPIO_PORT->B[5][5] = true; //GPIO5[5],CLOCK_READ
	LPC_GPIO_PORT->B[5][6] = false; //GPIO5[6],SRIN_READ

//lp1:
	for (ch = 0; ch < 32; ch++) {
		LPC_ADCHS->TRIGGER = 1;
# if 0
		val = *(uint32_t*) 0x400F0028; //last sample ch0;
		while (!(val & 0x1)) { //poll DONE bit on the second channel
			val = *(uint32_t*) 0x400F0028; //last sample ch0;
//		    val=*(uint32_t*)0x400F002c; //last sample ch1;
		}
		LPC_GPIO_PORT->B[5][5] = false;	 	  ///GPIO5[5],CLOCK_READ
		val = (val >> 6) & 0x0fff; //last sample ch0;
# else
		val = ilast*32+ch;
# endif
		evbuf.buf[ilast].adc[ch] = val;
		LPC_GPIO_PORT->B[5][5] = true; ///GPIO5[5],CLOCK_READ
	}
	evbuf.buf[ilast].T0 = ts0;
	evbuf.buf[ilast].T1 = ts1;
	evbuf.buf[ilast].coinc = coinc_bits;	//ABBA:
	evbuf.buf[ilast].flags = (evbuf.overwritten & 0x0000ffff) | (missed & 0x00ff0000);
    if((ts0 & 0x80000000) && !(ts0 & 0x40000000)) CorrectOscillator(ts0 & 0x3FFFFFFF);
	//end of loop 32
	LPC_GPIO_PORT->B[5][5] = false;  ///GPIO5[5],CLOCK_READ
	LPC_GPIO_PORT->B[0][7] = false; //GPIO0[7],RESETB_READ
	LPC_GPIO_PORT->B[0][7] = true; //GPIO0[7],RESETB_READ
	evbuf.i_last = ilast; //this is the marker that the event is ready
}

