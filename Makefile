default: build

build: 
	avr-gcc -mmcu=atmega328p -Os main.c -o main # outputs an .elf

burn: build
	avr-objcopy -O ihex -R .eeprom main main.hex # convert elf to .hex

	avrdude -F -V -c arduino -p m328p -P /dev/cu.usbmodem1101 -b 115200 -U flash:w:main.hex
