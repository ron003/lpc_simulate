 # This file (Makefile<M0>) was created by Ron Rechenmacher <ron@fnal.gov> on
 # Apr 25, 2020. "TERMS AND CONDITIONS" governing this file are in the README
 # or COPYING file. If you do not have such a file, one can be obtained by
 # contacting Ron or Fermi Lab in Batavia IL, 60510, phone: 630-840-3000.
 # $RCSfile: .emacs.gnu,v $
 # rev="$Revision: 1.34 $$Date: 2019/04/22 15:23:54 $";

# -D_DEFAULT_SOURCE declares signal stuff
#CC       = gcc -std=c11 -pedantic -D_DEFAULT_SOURCE -fsanitize=address
CC       = gcc -std=c11 -pedantic -D_DEFAULT_SOURCE # -I${TRACE_INC}
CFLAGS   = -g -Wall -Wextra
CPPFLAGS = -I.

headers     := $(wildcard *.h)
objects     := $(patsubst %.c,%.o,$(filter-out main_thread.c,$(wildcard *.c)))


main_thread: main_thread.c $(headers) $(objects)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o main_thread{,.c} $(objects) -pthread

clean:
	rm -f *.o main_thread
