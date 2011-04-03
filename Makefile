# Name: Makefile
# Author: Michiel van der Coelen
# date: 2011-03-31
# tabsize: 2
 
MMCU = atmega32
AVRDUDE_MCU = m32
AVRDUDE_PROGRAMMERID = usbasp
#AVRDUDE_PORT = 
F_CPU = 12000000
NAME = ServoControllerI2C


OBJECTS =  $(NAME).o 
CFLAGS = -DF_CPU=$(F_CPU) -std=c99 -Wall -Os -mmcu=$(MMCU) -I.
CC = avr-gcc
SIZE = avr-size
OBJCOPY = avr-objcopy

vpath %c ./src/

.PHONY: all clean test
all: bin/$(NAME).hex
	$(SIZE) bin/$(NAME).hex

#rebuild everything!
force: clean all

bin/$(NAME).hex: $(NAME).elf
	rm bin/$(NAME).hex
	$(OBJCOPY) -O ihex $(NAME).elf bin/$(NAME).hex
	rm $(OBJECTS) $(NAME).elf
	
$(NAME).elf: $(OBJECTS)
	$(CC) $(CFLAGS) -o $(NAME).elf $(OBJECTS)

#compile src files
%.o: %.c
		$(CC) $(CFLAGS) -c $< -o $@

%.o: %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $< -o $@


clean:
	rm -f $(OBJECTS) $(NAME).elf

program: bin\$(NAME).hex
	avrdude -c $(AVRDUDE_PROGRAMMERID) -p $(AVRDUDE_MCU) -U flash:w:bin/$(NAME).hex

test:
	avrdude -c $(AVRDUDE_PROGRAMMERID) -p $(AVRDUDE_MCU) -v