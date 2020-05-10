/*
 * CITIROC.c
 *
 *  Created on: Apr 27, 2015
 *      Author: lhep
 */
#include "board.h"
#include "CITIROC.h"

/* SCR pin definitions for RMII and MII modes */
#define EMAC_HIGHSLEW_INPUT  (MD_EHS | MD_PLN | MD_EZI | MD_ZI)
#define EMAC_HIGHSLEW_OUTPUT (MD_EHS | MD_PLN | MD_ZI)
#define EMAC_INPUT           (MD_PLN | MD_EZI)
#define EMAC_OUTPUT          (MD_PLN)


void Local_CITIROC_Init()
{
	// Initialize pins for CITIROC interface
	Chip_SCU_PinMuxSet(0x2, 0, (EMAC_OUTPUT | SCU_MODE_FUNC4)); //GPIO5[0], CLOCK_SR
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 5, 0);
	Chip_SCU_PinMuxSet(0x2, 1, (EMAC_OUTPUT | SCU_MODE_FUNC4)); //GPIO5[1],SRIN_SR
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 5, 1);
	Chip_SCU_PinMuxSet(0x2, 2, (EMAC_OUTPUT | SCU_MODE_FUNC4)); //GPIO5[2],LOAD_SC
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 5, 2);
	Chip_SCU_PinMuxSet(0x2, 3, (EMAC_OUTPUT | SCU_MODE_FUNC4)); //GPIO5[3],RESET_SR
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 5, 3);
	Chip_SCU_PinMuxSet(0x2, 4, (EMAC_OUTPUT | SCU_MODE_FUNC4)); //GPIO5[4],SELECT
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 5, 4);

	Chip_SCU_PinMuxSet(0x2, 5, (EMAC_OUTPUT | SCU_MODE_FUNC4)); //GPIO5[5],CLOCK_READ
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 5, 5);
	Chip_SCU_PinMuxSet(0x2, 6, (EMAC_OUTPUT | SCU_MODE_FUNC4)); //GPIO5[6],SRIN_READ
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 5, 6);
	Chip_SCU_PinMuxSet(0x2, 7, (EMAC_OUTPUT | SCU_MODE_FUNC0)); //GPIO0[7],RESETB_READ
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 7);

	Chip_SCU_PinMuxSet(0x2, 10, (EMAC_INPUT | SCU_MODE_FUNC0)); //GPIO0[14],OR32
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 14);
	Chip_SCU_PinMuxSet(0x2, 12, (EMAC_OUTPUT | SCU_MODE_FUNC0)); //GPIO1[12],PWR_ON
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 1, 12);

	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0x1, 12, 0); //PWR_ON


}

