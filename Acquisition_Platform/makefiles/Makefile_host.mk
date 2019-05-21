include ../makefiles/Makefile_common.mk

INCLUDE_DIRS = -I../src/common -I../src/host/
LIBS = -lpthread

CC = gcc

DEBUG_OPTS += -g

CCOPTS = -Wall -Ofast -std=gnu99 -Wstrict-prototypes $(LIBS) $(INCLUDE_DIRS) $(DEBUG_OPTS) $(SENSORS) $(COMMON_VARS)

HEADERS = host.h serial.h serial_communication.h
OBJS = host.o serial.o serial_communication.o
BINS = main

.phony: clean all

all: $(BINS)

%.o : ../src/common/%.c
	$(CC) $(CCOPTS) -c -o $@  $<

%.o : ../src/host/%.c
	$(CC) $(CCOPTS) -c -o $@  $<

%.o : ../src/host/serial/%.c
	$(CC) $(CCOPTS) -c -o $@  $<

main: ../src/host/main.c $(OBJS)
	$(CC) $(CCOPTS) -o $@ $^ $(LIBS)

clean:
	rm -rf *~  $(BINS) $(OBJS)
