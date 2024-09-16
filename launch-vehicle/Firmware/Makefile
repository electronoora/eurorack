# Target name
TARGET=launch_vehicle

# CPU clock speed in Hz and MCU type
F_CPU=16000000
MCUTYPE=atmega808
ATPACK=Atmel.ATmega_DFP.2.0.401

# MCU and ISP type for avrdude
ISPMCU=m808
ISP=jtag2updi

# Serial port of ISP
TTY=/dev/tty.usbserial-AB0N3K9O

# Paths to binaries
AVR-HOME=/opt/homebrew
AVR-GCC=$(AVR-HOME)/bin/avr-gcc
AVRDUDE=$(AVR-HOME)/bin/avrdude
AVR-OBJCOPY=$(AVR-HOME)/bin/avr-objcopy
AVR-SIZE=$(AVR-HOME)/bin/avr-size
WGET=$(AVR-HOME)/bin/wget

# Arguments passed to gcc
COPTS=-I. -I./atpack/include -B./atpack/gcc/dev/$(MCUTYPE) -std=gnu99 -Os
COPTS+=-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
COPTS+=-Wstrict-prototypes -Wall
COPTS+=-save-temps
COPTS+=-DF_CPU=$(F_CPU) -mmcu=$(MCUTYPE) -DLFSR_SEED=$(LFSR_SEED)

# Arguments passed to ld
LDOPTS=$(COPTS)


all: $(TARGET).hex

# Target for setting up the build environment
environment:
	mkdir atpack
	wget -c http://packs.download.atmel.com/$(ATPACK).atpack && \
	unzip -u -d atpack $(ATPACK).atpack 
	rm -f $(ATPACK).atpack 


clean:
	rm -f *~ *.o *.i *.s $(TARGET).elf $(TARGET).hex $(ATPACK).atpack

distclean: clean
	rm -rf atpack


# Targets for building and flashing firmware image
$(TARGET).o: $(TARGET).c
	$(AVR-GCC) $(COPTS) -c -o $(TARGET).o $(TARGET).c && \
	$(AVR-SIZE) $(TARGET).o
	
$(TARGET).elf: $(TARGET).o
	$(AVR-GCC) $(LDOPTS) -o $(TARGET).elf $(TARGET).o

$(TARGET).hex: $(TARGET).elf
	$(AVR-OBJCOPY) -j .text -j .data -O ihex $(TARGET).elf $(TARGET).hex

flash: $(TARGET).hex
	$(AVRDUDE) -p $(ISPMCU) -c $(ISP) -P $(TTY) -b 115200 -U flash:w:$(TARGET).hex:i -E reset -v

