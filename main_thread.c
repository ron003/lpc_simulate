/*  This file (main_thread.c) was created by Ron Rechenmacher <ron@fnal.gov> on
	Apr 25, 2020. "TERMS AND CONDITIONS" governing this file are in the README
	or COPYING file. If you do not have such a file, one can be obtained by
	contacting Ron or Fermi Lab in Batavia IL, 60510, phone: 630-840-3000.
	$RCSfile: .emacs.gnu,v $
	rev="$Revision: 1.34 $$Date: 2019/04/22 15:23:54 $";
	*/

#include <stdio.h>		/* printf */
#include <signal.h>		/* SIGUSR{1,2} */
#include <pthread.h>	/* pthread_ */
#include <unistd.h>		/* usleep */
#include <TRACE/trace.h>

uint8_t periph_mem[0xf8000];
static uint8_t m0m4shmem[0x20000];

void* M0_main(void*);
void* M4_main(void*);

int main( /*int	argc, char	*argv[]*/ )
{
	sigset_t set;
	pthread_t threads[2];
	enum { m0_thread, m4_thread };
	void *evbufp =  &m0m4shmem[0x10000];
	void *EvRate = &m0m4shmem[0x100];
	void *DAQ_Enabled = &m0m4shmem[0x200];

	TRACE(13,"m0m4shmem is at %p",m0m4shmem );
	sigemptyset(&set);
	sigaddset(&set, SIGUSR1);
	sigaddset(&set, SIGUSR2);
	sigprocmask(SIG_BLOCK, &set, NULL);
	// The first 3 ptrs are from *FEB_test*.c
	*(void**)&m0m4shmem[sizeof(void*)*0] = evbufp;
	*(void**)&m0m4shmem[sizeof(void*)*1] = EvRate;
	*(void**)&m0m4shmem[sizeof(void*)*2] = DAQ_Enabled;

    // The next 3 ptrs are from M4*main.c

	// Seem the M4 core is really the main core of the whole Micro-Controller.
	// The M4_main seems to initialize everything.
	pthread_create(&threads[m4_thread],NULL, M4_main, m0m4shmem);
	usleep(500000); // make sure m4 starts and is waiting for 
	// M0 core does the ethernet communications
	pthread_create(&threads[m0_thread],NULL, M0_main, m0m4shmem);

	// https://www.linuxprogrammingblog.com/all-about-linux-signals?page=11

	unsigned mod=4;
	for (unsigned uu=0; uu<100000; ++uu) {
		if ((uu%mod)==(mod-1)){
			if (uu==18175) mod=4096;
			else           mod=4;
			TRACE(7,"uu=%u send irq SIGUSR1 -- MX_CORE_IRQ - interrupt sleep to cause TrasmitEventBuffer",uu);
			pthread_kill( threads[m0_thread], SIGUSR1 );
			usleep(0);
		}
		TRACE(7,"send irq SIGUSR2 -- GPIO0_IRQ - upstream event creation");
		pthread_kill( threads[m4_thread], SIGUSR2 );
		usleep(0);
	}
	TRACE(1,"Done with SIGUSR1,SIGUSR2");
	exit(0);

	pthread_join(threads[m4_thread], NULL);
	pthread_join(threads[m0_thread], NULL);
	return (0);
}   /* main */
