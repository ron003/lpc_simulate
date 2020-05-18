/*
 */

/** Define DEBUG_ENABLE to enable IO via the DEBUGSTR, DEBUGOUT, and
 DEBUGIN macros. If not defined, DEBUG* functions will be optimized
 out of the code at build time.
 */
//#define DEBUG_ENABLE
/** Define DEBUG_SEMIHOSTING along with DEBUG_ENABLE to enable IO support
 via semihosting. You may need to use a C library that supports
 semihosting with this option.
 */
//#define DEBUG_SEMIHOSTING

#define HAVE_LPC43XX_M0_IRQn_Type
#include "board.h"
#include <stdio.h>
#include <string.h>
//#include "ethernet.h"
#include "FEBDTP.h"
#include "CITIROC.h"
#include "EVENT.h"
#include "HSADC.h"
#include <cr_section_macros.h>
#include "app_multicore_cfg.h"
#include "FPGA.h"
#include "pinint_18xx_43xx.h"

#include "chip_clocks.h"
void Chip_SetupCoreClock(CHIP_CGU_CLKIN_T clkin, uint32_t core_freq, bool setbase);

#define TRACE_NAME trace_path_components(__FILE__,1)

void Board_Init(void);
void Local_CITIROC_Init(void);
uint32_t SysTick_Config(uint32_t ticks);
#include "system_LPC43xx.h"
#define __CMSIS_H_
#include "cmsis_43xx.h"

uint32_t SystemCoreClock;

Evbuf_t evbuf; // main event buffer, see EVENT.h

static uint8_t*m0m4shmem;
#define M0M4SHMEM m0m4shmem     //0x10088000
#define M0M4STOP (M0M4SHMEM+sizeof(addr_t)*3) //flags for iter-CPU communications: to stop M4 in RAM code for safe FLASH programming
#define M4M0STOPED (M0M4SHMEM+sizeof(addr_t)*4) //flags for iter-CPU communications: tells to M0 that M4 is in safe RAM code loop
#define M4M0CFGFPGA (M0M4SHMEM+sizeof(addr_t)*5) //flags for iter-CPU communications: tells to M0 that M4 is in safe RAM code loop

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
uint8_t SwRegVal[16];

uint8_t framebuf[2000];
uint16_t framesize = 128 - 14;
//uint8_t dstmacaddr[6];

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
int tick = 0;
int tick2 = 0;
float EvRate = 0; // global variable for current trigger rate evaluation
int EvCounter = 0; // global counter variable for current trigger rate evaluation
static int DAQ_Enabled = 0; // global DAQ enable
int FPGA_programmed = 0;


__RAMFUNC(RAM2) void StartDownloadFirmwareOnFPGA()
{
	Chip_SSP_WriteFrames_Blocking(LPC_SSP1, (addr_t)0x14080000, 0x80000);
}
__RAMFUNC(RAM2) void SysTick_Handler(void) {
	if (tick++ > 500) {
		tick = 0;
		EvRate = 2.0 * EvCounter; //in Herz
		EvCounter = 0;
		//   	Board_LED_Toggle(0);
		__DSB();
		__SEV();
//    	LPC_CREG->M4TXEVENT = 1;
		//sent interrupt to M0APP to toggle LED
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0x1, 13, 0); //TS_ENA //security measure against hanging
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0x1, 13, 1); //TS_ENA
		tick2++;
		if (FPGA_programmed==0)
		{
			StartDownloadFirmwareOnFPGA();
			FPGA_programmed=1;
		}
	}

	if (tick2>2)
	{

	}

}

__RAMFUNC(RAM2) void GPIO0_IRQHandler(void) {
	TRACE(3,"GPIO0_IRQHandler called");
	//	 Board_LED_Toggle(0);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, 0xff);
	if(DAQ_Enabled==1){
	LPC_GPIO_PORT->B[0][8] = true; //fast LED on
	ProcessEvent();
	EvCounter++;
	}
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0x1, 13, 0); //TS_ENA //reset HOLD FlipFlop
	LPC_GPIO_PORT->B[0][8] = false; //fast LED off
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0x1, 13, 1); //TS_ENA

}

static void __disable_irq()
{
	sigset_t set;
	TRACE(3,"__disable_irq() SIG_BLOCKing SIGUSR2");
	sigemptyset(&set);
	sigaddset(&set, SIGUSR2);
	pthread_sigmask(SIG_BLOCK, &set, NULL);
}
static void __enable_irq()
{
	sigset_t unblock_set, block_add_set;
	struct sigaction  new_sigaction_s={0};
	TRACE(3,"__enable_irq() SIG_UNBLOCKing SIGUSR2 - handler GPIO0_IRQHandler");
	sigemptyset(&unblock_set);
	sigaddset(&unblock_set, SIGUSR2);
	sigemptyset(&block_add_set);
	new_sigaction_s.sa_mask = block_add_set;
	new_sigaction_s.sa_handler = (handler_t)GPIO0_IRQHandler;
	sigaction( SIGUSR2, &new_sigaction_s, NULL );
	pthread_sigmask(SIG_UNBLOCK, &unblock_set, NULL);
}



__RAMFUNC(RAM2) int M4_main(void *arg) {
	M0M4SHMEM = (uint8_t*)arg;
	*((addr_t*) (M0M4SHMEM + sizeof(addr_t)*0)) = (addr_t)(&evbuf); // store address of the event buffer for M0 core
	*((addr_t*) (M0M4SHMEM + sizeof(addr_t)*1)) = (addr_t)(&EvRate); // store address of the event rate counter for M0 core
	*((addr_t*) (M0M4SHMEM + sizeof(addr_t)*2)) = (addr_t)(&DAQ_Enabled); // store address of the DAQ Enable flag for M0 core

	/* Initialize board and chip */
//	Chip_SetupXtalClocking();
//	Chip_SetupCoreClock(CLKIN_CRYSTAL, 200000000, true); //set core clock to 140 MHz only to reduce power
	Chip_SetupCoreClock(CLKIN_CRYSTAL, 160000000, true); //set core clock to 160 MHz only to reduce power
	Chip_Clock_SetBaseClock(CLK_BASE_MX, CLKIN_MAINPLL, true, false); //set bus clock to the same freq
	Chip_Clock_SetBaseClock(CLK_BASE_PERIPH, CLKIN_MAINPLL, true, false); //set M0 bus clock to the same freq
	Chip_Clock_SetBaseClock(CLK_BASE_APB3, CLKIN_MAINPLL, true, false); //set DAC clock
	Chip_Clock_EnableBaseClock(CLK_BASE_APB3);

	SystemCoreClockUpdate();
	Board_Init();
//		CLK_BASE_SAFE,		/*!< Base clock for WDT oscillator, IRC input only */
//		CLK_BASE_USB0,		/*!< Base USB clock for USB0, USB PLL input only */
//		CLK_BASE_PERIPH,	/*!< Base clock for SGPIO */
//		CLK_BASE_USB1,		/*!< Base USB clock for USB1 */
//		CLK_BASE_MX,		/*!< Base clock for CPU core */
//		CLK_BASE_SPIFI,		/*!< Base clock for SPIFI */
//		CLK_BASE_SPI,		/*!< Base clock for SPI */
//		CLK_BASE_PHY_RX,	/*!< Base clock for PHY RX */
//		CLK_BASE_PHY_TX,	/*!< Base clock for PHY TX */
//		CLK_BASE_APB1,		/*!< Base clock for APB1 group */
//		CLK_BASE_APB3,		/*!< Base clock for APB3 group */
//		CLK_BASE_LCD,		/*!< Base clock for LCD pixel clock */
//		CLK_BASE_ADCHS,		/*!< Base clock for ADCHS */
//		CLK_BASE_LCD,		/*!< Base clock for SDIO */
//		CLK_BASE_SSP0,		/*!< Base clock for SSP0 */
//		CLK_BASE_SSP1,		/*!< Base clock for SSP1 */
//		CLK_BASE_UART0,		/*!< Base clock for UART0 */
//		CLK_BASE_UART1,		/*!< Base clock for UART1 */
//		CLK_BASE_UART2,		/*!< Base clock for UART2 */
//		CLK_BASE_UART3,		/*!< Base clock for UART3 */
//		CLK_BASE_OUT,		/*!< Base clock for CLKOUT pin */
//		CLK_BASE_RESERVED4,
//		CLK_BASE_RESERVED5,
//		CLK_BASE_RESERVED6,
//		CLK_BASE_RESERVED7,
//		CLK_BASE_APLL,		/*!< Base clock for audio PLL */
//		CLK_BASE_CGU_OUT0,	/*!< Base clock for CGUOUT0 pin */
//		CLK_BASE_CGU_OUT1,	/*!< Base clock for CGUOUT1 pin */
//		CLK_BASE_LAST,
//		CLK_BASE_NONE = CLK_BASE_LAST

	Chip_Clock_DisableBaseClock(CLK_BASE_USB0);
	Chip_Clock_DisableBaseClock(CLK_BASE_USB1);
	Chip_Clock_DisableBaseClock(CLK_BASE_LCD);
	Chip_Clock_DisableBaseClock(CLK_BASE_UART0);
	Chip_Clock_DisableBaseClock(CLK_BASE_UART1);
	Chip_Clock_DisableBaseClock(CLK_BASE_UART2);
	Chip_Clock_DisableBaseClock(CLK_BASE_UART3);
	Chip_Clock_DisableBaseClock(CLK_BASE_OUT);
	Chip_Clock_DisableBaseClock(CLK_BASE_APLL);
	Chip_Clock_DisableBaseClock(CLK_BASE_CGU_OUT0);
	Chip_Clock_DisableBaseClock(CLK_BASE_CGU_OUT1);
	Chip_Clock_DisableBaseClock(CLK_BASE_APB1);
	/* enable clocks and pinmux */
	//USB_init_pin_clk();

	Local_CITIROC_Init();
	Local_FPGA_Init();
	/* Setup high speed ADC */
	Local_HSADC_Init();
	/* Enable and setup SysTick Timer at a periodic rate */
	SysTick_Config(SystemCoreClock / 1000);

	/* Enable Group GPIO interrupt 0 for OR32 input (configured in CITIROC.c)*/
	NVIC_EnableIRQ(PIN_INT0_IRQn);
	//Reset test 74HC64D
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0x1, 13, 0); //TS_ENA //user as HOLD reset for tests, active low
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0x1, 13, 1); //TS_ENA //user as HOLD reset for tests, active low
// DAC for VTCXO control
//	Chip_SCU_PinMuxSet(0x4, 4, (SCU_MODE_FUNC0)); //DAC output gpio2-4
	//Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 2, 4);
//	Chip_SCU_DAC_Analog_Config();
//	Chip_Clock_EnableOpts(CLK_APB3_DAC, true, true, 1);
//	Chip_DAC_SetBias(LPC_DAC, DAC_MAX_UPDATE_RATE_400kHz);
	Board_DAC_Init(LPC_DAC);
	/* DAC Init */
	Chip_DAC_Init(LPC_DAC);
	/* set time out for DAC*/
	Chip_DAC_SetDMATimeOut(LPC_DAC, 0xFFFF);
	Chip_DAC_ConfigDAConverterControl(LPC_DAC, (DAC_CNT_ENA | DAC_DMA_ENA));
	Chip_DAC_UpdateValue(LPC_DAC, 489); //10 ADC~=180ns


	evbuf.numevts = 0;
	evbuf.overwritten = 0; // number of overwritten (lost) events
	evbuf.i_first = 0;
	evbuf.i_last = 0xffff; //(equal to -1)
	*((uint8_t*)M0M4STOP)=0; //init IPC flags
	*((uint8_t*)M4M0STOPED)=0;
	/* Time to Start M0APP */
	if (M0Image_Boot(CPUID_M0APP, (uint32_t) BASE_ADDRESS_M0APP) < 0) {
		DEBUGSTR("Unable to BOOT M0APP Core!");
	}
    volatile uint8_t STOPCMDFROMM0=0;
	while(1) {
		STOPCMDFROMM0=*((uint8_t*)M0M4STOP);
		TRACE(3,"STOPCMDFROMM0=%u    waiting for STOPCMDFROMM0>0",STOPCMDFROMM0);
        if(STOPCMDFROMM0>0)
        {
        	__disable_irq();
        	*((uint8_t*)M4M0STOPED)=1;
        	while(STOPCMDFROMM0>0){
				STOPCMDFROMM0=*((uint8_t*)M0M4STOP);
			}
        	*((uint8_t*)M4M0STOPED)=0;
        	__enable_irq();
        }
        //__WFI();
		TRACE(3,"M4_main while(1) -- sleeping 1"); usleep(1000000);
	}
	return 0;
}

void      Chip_Clock_DisableBaseClock(CHIP_CGU_BASE_CLK_T BaseClock)
{TRACE(3,"Chip_Clock_DisableBaseClock(%d) called.",BaseClock);}

uint32_t  Chip_SSP_WriteFrames_Blocking(LPC_SSP_T *pSSP, uint8_t *buffer, uint32_t buffer_len)
{TRACE(3,"Chip_SSP_WriteFrames_Blocking(%p, %p. %u) called.", (void*)pSSP, (void*)buffer, buffer_len);
 return (0);}

void      Chip_SetupCoreClock(CHIP_CGU_CLKIN_T clkin, uint32_t core_freq, bool setbase)
{TRACE(3,"Chip_SetupCoreClock(clkin=%d, core_freq=%u, setbase=%d) called.",clkin, core_freq, setbase);}

void    Chip_Clock_SetBaseClock(CHIP_CGU_BASE_CLK_T BaseClock, CHIP_CGU_CLKIN_T Input, bool autoblocken, bool powerdn)
{TRACE(3,"Chip_Clock_SetBaseClock(BaseClock=%d, Input=%d, autoblocken=%d powerdn=%d)",BaseClock,Input,autoblocken,powerdn);}

void      Chip_Clock_EnableBaseClock(CHIP_CGU_BASE_CLK_T BaseClock)
{TRACE(3,"Chip_Clock_EnableBaseClock(BaseClock=%d) called.", BaseClock);}

void      Board_Init(void)
{TRACE(3,"Board_Init() called.");}

void      Local_FPGA_Init()
{TRACE(3,"Local_FPGA_Init() called.");}

void      Local_HSADC_Init()
{TRACE(3,"Local_HSADC_Init() called.");}

uint32_t  SysTick_Config(uint32_t ticks)
{TRACE(3,"SysTick_Config(ticks=%u) called.", ticks);
 return (0);}

void      Board_DAC_Init(LPC_DAC_T *pDAC)
{TRACE(3,"Board_DAC_Init(pDAC=%p) called.", (void*)pDAC);}

void      Chip_DAC_Init(LPC_DAC_T *pDAC)
{TRACE(3,"Chip_DAC_Init(pDAC=%p) called.", (void*)pDAC);}

int       M0Image_Boot(CPUID_T cpu, uint32_t base_addr)
{TRACE(3,"M0Image_Boot(cpu=%d, base_addr=%u) called.", cpu, base_addr);
 return (0);}
