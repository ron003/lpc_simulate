/*  This file (main_thread.c) was created by Ron Rechenmacher <ron@fnal.gov> on
	Apr 25, 2020. "TERMS AND CONDITIONS" governing this file are in the README
	or COPYING file. If you do not have such a file, one can be obtained by
	contacting Ron or Fermi Lab in Batavia IL, 60510, phone: 630-840-3000.
	$RCSfile: .emacs.gnu,v $
	rev="$Revision: 1.34 $$Date: 2019/04/22 15:23:54 $";
	*/

#include <stdio.h>		/* printf */
#include <pthread.h>	/* pthread_ */
#include <unistd.h>		/* usleep */
#include <TRACE/trace.h>

uint8_t periph_mem[0xf8000];
static uint8_t m0m4shmem[0x20000];

void* M0_main(void*);
void* M4_main(void*);

int main( /*int	argc, char	*argv[]*/ )
{
	pthread_t threads[2];

	void *evbufp =  &m0m4shmem[0x10000];
	void *EvRate = &m0m4shmem[0x100];
	void *DAQ_Enabled = &m0m4shmem[0x200];

	// The first 3 ptrs are from *FEB_test*.c
	*(void**)&m0m4shmem[sizeof(void*)*0] = evbufp;
	*(void**)&m0m4shmem[sizeof(void*)*1] = EvRate;
	*(void**)&m0m4shmem[sizeof(void*)*2] = DAQ_Enabled;

    // The next 3 ptrs are from M4*main.c

	// Seem the M4 core is really the main core of the whole Micro-Controller.
	// The M4_main seems to initialize everything.
	pthread_create(&threads[1],NULL, M4_main, m0m4shmem);
	usleep(500000);
	// M0 core does the ethernet communications
	pthread_create(&threads[0],NULL, M0_main, m0m4shmem);
	
	pthread_join(threads[0], NULL);
	pthread_join(threads[1], NULL);
	return (0);
}   /* main */
