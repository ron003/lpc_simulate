/*  This file (dac_18xx_43xx.c) was created by Ron Rechenmacher <ron@fnal.gov> on
	Apr 27, 2020. "TERMS AND CONDITIONS" governing this file are in the README
	or COPYING file. If you do not have such a file, one can be obtained by
	contacting Ron or Fermi Lab in Batavia IL, 60510, phone: 630-840-3000.
	$RCSfile: .emacs.gnu,v $
	rev="$Revision: 1.34 $$Date: 2019/04/22 15:23:54 $";
	*/

#include "board.h"
//#include "dac_18xx_43xx.h"
#include <TRACE/trace.h>

void      Chip_DAC_UpdateValue(LPC_DAC_T *pDAC, uint32_t dac_value)
{TRACE(3,"Chip_DAC_UpdateValue(pDAC=%p, dac_value=%u) called.", (void*)pDAC, dac_value);}
