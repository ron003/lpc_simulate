#ifndef BOARD_H
#define BOARD_H

#include <TRACE/trace.h>
#define DEBUGOUT(...) TRACE(3,__VA_ARGS__)
#define DEBUGSTR(ss)  TRACE(3,ss)
#include <signal.h>				// sigset_t
#include <stdint.h>
typedef unsigned long int ulong;
typedef _Bool bool;
#define EMAC_ETH_MAX_FLEN (1536)

#define __IO volatile
#define __I  volatile
#define __O  volatile
#define __DSB()
#define __SEV()

enum { false, true };

#define Board_LED_Toggle(x) TRACE(3,"Board_LED_Toggle(%d)", x)
__attribute__((__unused__))
static void ENET_SendFrameBlocking(uint8_t*buf,uint16_t size){TRACE(3,"ENET_SendFrameBlocking(%p,%u)",buf,size);}
__attribute__((__unused__))
static void ENET_SendFrameNonBlocking(uint8_t*buf,uint16_t size){
	TRACE(3,"ENET_SendFrameNonBlocking(%p,%u)",buf,size);}
#define SystemCoreClockUpdate()   TRACE(3,"SystemCoreClockUpdate()")
#define NVIC_EnableIRQ(x)         TRACE(3,"NVIC_EnableIRQ(%u)",x)
#define Board_SSP_Init(x)         TRACE(3,"Board_SSP_Init(%p)",(void*)x)
#define Board_ENET_Init()         TRACE(3,"Board_ENET_Init()")

typedef unsigned char * addr_t;
typedef void (*handler_t)(int);

// From workspace2/lpc_chip_43xx_m0/inc/config_m0app/cmsis_43xx_m0app.h
#ifndef HAVE_LPC43XX_M0_IRQn_Type
typedef enum {
        /* -------------------------  Cortex-M0 Processor Exceptions Numbers  ----------------------------- */
        Reset_IRQn                        = -15,/*!<   1  Reset Vector, invoked on Power up and warm reset */
        NonMaskableInt_IRQn               = -14,/*!<   2  Non maskable Interrupt, cannot be stopped or preempted */
        HardFault_IRQn                    = -13,/*!<   3  Hard Fault, all classes of Fault */
        SVCall_IRQn                       = -5, /*!<  11  System Service Call via SVC instruction */
        DebugMonitor_IRQn                 = -4, /*!<  12  Debug Monitor                    */
        PendSV_IRQn                       = -2, /*!<  14  Pendable request for system service */
        SysTick_IRQn                      = -1, /*!<  15  System Tick Timer           */

        /* ---------------------------  LPC18xx/43xx Specific Interrupt Numbers  ------------------------------- */
        RTC_IRQn                          =   0,/*!<   0  RTC                              */
        M4_IRQn                           =   1,/*!<   1  M4 Core interrupt                */
        DMA_IRQn                          =   2,/*!<   2  DMA                              */
        RESERVED1_IRQn                    =   3,/*!<   3                                   */
        FLASHEEPROM_IRQn                  =   4,/*!<   4  ORed Flash Bank A, B, EEPROM     */
        ATIMER_IRQn                       =   4,/*!<   4  ATIMER ORed with Flash/EEPROM    */
        ETHERNET_IRQn                     =   5,/*!<   5  ETHERNET                         */
        SDIO_IRQn                         =   6,/*!<   6  SDIO                             */
        LCD_IRQn                          =   7,/*!<   7  LCD                              */
        USB0_IRQn                         =   8,/*!<   8  USB0                             */
        USB1_IRQn                         =   9,/*!<   9  USB1                             */
        SCT_IRQn                          =  10,/*!<  10  SCT                              */
        RITIMER_IRQn                      =  11,/*!<  11  ORed RITIMER, WWDT               */
        WWDT_IRQn                         =  11,/*!<  11  ORed RITIMER, WWDT               */
        TIMER0_IRQn                       =  12,/*!<  12  TIMER0                           */
        GINT1_IRQn                        =  13,/*!<  13  GINT1                            */
        PIN_INT4_IRQn                     =  14,/*!<  14  GPIO 4                           */
        TIMER3_IRQn                       =  15,/*!<  15  TIMER3                           */
        MCPWM_IRQn                        =  16,/*!<  16  MCPWM                            */
        ADC0_IRQn                         =  17,/*!<  17  ADC0                             */
        I2C0_IRQn                         =  18,/*!<  18  ORed I2C0, I2C1                  */
        I2C1_IRQn                         =  18,/*!<  18  ORed I2C0, I2C1                  */
        SGPIO_INT_IRQn                    =  19,/*!<  19  SGPIO                            */
        SPI_INT_IRQn                      =  20,/*!<  20  ORed SPI/DAC                     */
        DAC_IRQn                          =  20,/*!<  20  ORed SPI/DAC                     */
        ADC1_IRQn                         =  21,/*!<  21  ADC1                             */
        SSP0_IRQn                         =  22,/*!<  22  ORed SSP0, SSP1                  */
        SSP1_IRQn                         =  22,/*!<  22  ORed SSP0, SSP1                  */
        EVENTROUTER_IRQn                  =  23,/*!<  23  EVENTROUTER                      */
        USART0_IRQn                       =  24,/*!<  24  USART0                           */
        UART1_IRQn                        =  25,/*!<  25  UART1                            */
        USART2_IRQn                       =  26,/*!<  26  ORed USART2/C_CAN1               */
        C_CAN1_IRQn                       =  26,/*!<  29  ORed USART2/C_CAN1               */
        USART3_IRQn                       =  27,/*!<  27  USART3                           */
        I2S0_IRQn                         =  28,/*!<  28  ORed I2S0/I2S1/QEI               */
        I2S1_IRQn                         =  28,/*!<  29  ORed I2S0/I2S1/QEI               */
        QEI_IRQn                          =  28,/*!<  29  ORed I2S0/I2S1/QEI               */
        C_CAN0_IRQn                       =  29,/*!<  29  C_CAN0                           */
        ADCHS_IRQn                        =  30,/*!<  30  ADCHS interrupt                  */
        M0SUB_IRQn                        =  31,/*!<  31  M0SUB                            */
} LPC43XX_M0_IRQn_Type;
#endif


// From workspace2/lpc_chip_43xx_m0/inc/chip_lpc43xx.h

extern  uint8_t periph_mem[0xf8000];
#define LPC_ETHERNET_BASE         (&periph_mem[0x10000]) //0x40010000
#define LPC_SPIFI_BASE            (&periph_mem[0x03000]) //0x40003000
#define LPC_CREG_BASE             (&periph_mem[0x43000]) //0x40043000
#define LPC_WWDT_BASE             (&periph_mem[0x80000]) //0x40080000
#define LPC_SSP0_BASE             (&periph_mem[0x83000]) //0x40083000
#define LPC_SSP1_BASE             (&periph_mem[0xc5000]) //0x400C5000
#define LPC_SCU_BASE              (&periph_mem[0x86000]) //0x40086000
#define LPC_PIN_INT_BASE          (&periph_mem[0x87000]) //0x40087000
#define LPC_DAC_BASE              (&periph_mem[0xe1000]) //0x400E1000
#define LPC_ADCHS_BASE            (&periph_mem[0xf0000]) //0x400F0000
#define LPC_GPIO_PORT_BASE        (&periph_mem[0xf4000]) //0x400F4000

#define LPC_ETHERNET              ((LPC_ENET_T             *) LPC_ETHERNET_BASE)
#define LPC_CREG                  ((LPC_CREG_T             *) LPC_CREG_BASE)
#define LPC_WWDT                  ((LPC_WWDT_T             *) LPC_WWDT_BASE)
#define LPC_SSP0                  ((LPC_SSP_T              *) LPC_SSP0_BASE)
#define LPC_SSP1                  ((LPC_SSP_T              *) LPC_SSP1_BASE)
#define LPC_SCU                   ((LPC_SCU_T              *) LPC_SCU_BASE)
#define LPC_GPIO_PIN_INT          ((LPC_PIN_INT_T          *) LPC_PIN_INT_BASE)
#define LPC_DAC                   ((LPC_DAC_T              *) LPC_DAC_BASE)
#define LPC_ADCHS                 ((LPC_HSADC_T            *) LPC_ADCHS_BASE)
#define LPC_GPIO_PORT             ((LPC_GPIO_T             *) LPC_GPIO_PORT_BASE)



#include "lpc_types.h"
#define INLINE inline
#define CHIP_LPC43XX			// defined in workspace2/lpc_chip_43xx_m0/inc/config_m0app/sys_config.h
#include "scu_18xx_43xx.h"
#include "creg_18xx_43xx.h"
#include "wwdt_18xx_43xx.h"
#include "ssp_18xx_43xx.h"
#include "enet_18xx_43xx.h"
#include "gpio_18xx_43xx.h"
#include "dac_18xx_43xx.h"

// needed just for M4
#include "clock_18xx_43xx.h"
#include "hsadc_18xx_43xx.h"    // needs clock_18xx_43xx.h to be included earlier

void Board_DAC_Init(LPC_DAC_T *pDAC);

#endif /* BOARD_H */
