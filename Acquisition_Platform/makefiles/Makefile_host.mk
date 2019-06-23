include ../makefiles/Makefile_common.mk

INCLUDE_DIRS = -I../src/common -I../src/host/
LIBS = -lpthread -lm

CC = gcc

DEBUG_OPTS += -g

CCOPTS = -Wall -Ofast -std=gnu99 -Wstrict-prototypes $(LIBS) $(INCLUDE_DIRS) $(DEBUG_OPTS) $(SENSORS) $(ODOM) $(COMMON_VARS)

HEADERS = host.h serial.h serial_communication.h matrix.h	kalman_filter.h
OBJS = host.o serial.o serial_communication.o matrix.o	kalman_filter.o
BINS = main

.phony: clean all

all: $(BINS)

%.o : ../src/common/%.c
	$(CC) $(CCOPTS) -c -o $@  $<

%.o : ../src/host/%.c
	$(CC) $(CCOPTS) -c -o $@  $<

%.o : ../src/host/serial/%.c
	$(CC) $(CCOPTS) -c -o $@  $<

%.o : ../src/host/kalman_filter/%.c
	$(CC) $(CCOPTS) -c -o $@  $<

main: ../src/host/main.c $(OBJS)
	$(CC) $(CCOPTS) -o $@ $^ $(LIBS)

clean:
	rm -rf *~  $(BINS) $(OBJS)
