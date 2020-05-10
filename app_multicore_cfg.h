/*
 * @brief LPC43XX Multicore application configuration file
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#ifndef __APP_MULTICORE_CFG_H_
#define __APP_MULTICORE_CFG_H_

#include "board.h"

typedef enum CPUID {
	CPUID_MIN,
	CPUID_M4 = CPUID_MIN,
	CPUID_M0APP,
	CPUID_M0SUB,
	CPUID_MAX
} CPUID_T;

#define CPUID_CURR     CPUID_M4


/* Size of applications in flash */
#define IMG_SZ_M4      (32 * 1024)
#define IMG_SZ_M0APP   (32 * 1024)
#define IMG_SZ_M0SUB   (32 * 1024)

#define SPIFI_BASE_ADDR     0x14000000	/* SPIFI BASE ADDR */


#define BASE_ADDRESS_M0APP     (SPIFI_BASE_ADDR + IMG_SZ_M4)

#define BASE_ADDRESS_M0SUB     (SPIFI_BASE_ADDR + IMG_SZ_M4 + IMG_SZ_M0APP)

/**
 * @brief		Boot M0 Image stored in @a base_addr
 * @param		cpu		: ID of the cpu to boot
 * @param		base_addr: Address of the image
 * @return		0 on success < 0 on failure
 */
int M0Image_Boot(CPUID_T cpu, uint32_t base_addr);

#endif /* __APP_MULTICORE_CFG_H_ */
