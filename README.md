# mrgtimer\_firmware
firmware for Manitoba Robot Games timer board

# Prerequisites

Use arduino 1.8... newer versions might work too

Install DFRobot LCD library.  In Arduino IDE, go to "Tools > Manage Libraries..."

Search for DFRobot LCD and install DFRobot\_RGBLCD1602 v2.0.0

# Directories

## old\_sketches/race\_timer

This is the original Arduino sketch by Bruce

## old\_sketches/mrg\_timer

This is some work done by Troy, wholly ignorant of the previous work done by Bruce

## mrgtimer\_firmware

This is the functioning sketch, a combination of the above two works

## kicad

board design files

# Compilation

open the mrgtimer\_firmware project in the arduino ide

this can be done with `make gui` (read the Makefile for instructions)

ensure the board type is set to 'Arduino Nano' (atmega328p variant)

# Compilation w/ arduino-cli

Install arduino cli
install avr core: ~/Software/arduino-cli core install arduino:avr
now you can use the following:
make verify
make upload

