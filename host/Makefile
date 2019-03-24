CCOPTS= -Wall -g -std=gnu99 -Wstrict-prototypes
LIBS= 
CC=gcc
AR=ar


BINS = host

OBJS = serial.o

HEADERS = serial.h

%.o: %.c $(HEADERS)
	$(CC) $(CCOPTS) -c -o $@  $<

.phony: clean all


all: $(BINS) 

host: host.c $(OBJS) 
	$(CC) $(CCOPTS) -o $@ $^ $(LIBS)

clean:
	rm -rf *.o *~  $(BINS)
