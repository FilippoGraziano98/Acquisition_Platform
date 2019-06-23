include ../makefiles/Makefile_common.mk

CC=avr-gcc
SRCDIR = ../src/firmware
INCLUDE_DIRS=-I../src/common -I$(SRCDIR)


CC_OPTS=-Wall --std=gnu99 -DF_CPU=16000000UL -O3 -funsigned-char -funsigned-bitfields  -fshort-enums -Wall -Wstrict-prototypes -mmcu=atmega2560 $(INCLUDE_DIRS)  -D__AVR_3_BYTE_PC__ $(DEBUG_OPTS) $(SENSORS) $(ODOM) $(COMMON_VARS)
AS_OPTS=-x assembler-with-cpp $(CC_OPTS)

AVRDUDE = avrdude

# com1 = serial port. Use lpt1 to connect to parallel port.
AVRDUDE_PORT = /dev/ttyACM0    # programmer connected to serial device
#AVRDUDE_PORT = /dev/ttyS4

AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET):i
AVRDUDE_FLAGS = -p m2560 -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) -b 115200
AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)
AVRDUDE_FLAGS += -D -q -V -C /usr/share/arduino/hardware/tools/avr/../avrdude.conf
AVRDUDE_FLAGS += -c wiring




HEADERS = uart.h i2c.h eeprom.h i2c_communication.h uart_packets.h packet_handler.h encoder.h encoder_odometry.h imu.h imu_odometry.h acquisition_platform.h #kalman_filter.h
OBJS = uart.o i2c.o eeprom.o i2c_communication.o uart_packets.o packet_handler.o encoder.o encoder_odometry.o imu.o imu_odometry.o acquisition_platform.o #kalman_filter.o

BINS = main.elf
FIRMWARE = main.hex


.phony:	clean all

all: $(BINS) $(FIRMWARE)

%.o: $(SRCDIR)/%.c
	$(CC) $(CC_OPTS) -c  -o $@ $<

%.o: $(SRCDIR)/avr_common/%.c
	$(CC) $(CC_OPTS) -c  -o $@ $<

%.o: $(SRCDIR)/packets/%.c
	$(CC) $(CC_OPTS) -c  -o $@ $<

%.o: $(SRCDIR)/encoder/%.c
	$(CC) $(CC_OPTS) -c  -o $@ $<
	
%.o: $(SRCDIR)/imu/%.c
	$(CC) $(CC_OPTS) -c  -o $@ $<
	
%.o: $(SRCDIR)/kalman_filter/%.c
	$(CC) $(CC_OPTS) -c  -o $@ $<
	
%.elf: %.o $(OBJS)
	$(CC) $(CC_OPTS) -o $@ $< $(OBJS) $(LIBS)

%.hex: %.elf
	avr-objcopy -O ihex -R .eeprom $< $@
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$@:i #$(AVRDUDE_WRITE_EEPROM) 

upload:
	avr-objcopy -O ihex -R .eeprom main.elf $(FIRMWARE)
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$(FIRMWARE):i #$(AVRDUDE_WRITE_EEPROM) 
	

clean:	
	rm -rf $(OBJS) $(BINS) *.hex *~ *.o

.SECONDARY:	$(OBJS)
