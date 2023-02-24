# mrgtimer\_firmware
firmware for Manitoba Robot Games timer board

# prerequisites

Use arduino 1.8... newer versions might work too

Install DFRobot LCD library.  In Arduino IDE, go to "Tools > Manage Libraries..."

Search for DFRobot LCD and install DFRobot\_RGBLCD1602 v2.0.0

# Directories

## race\_timer

This is the original Arduino sketch by Bruce

## mrg\_timer

This is some work done by Troy, wholly ignorant of the previous work done by bruce

## mrgtimer\_firmware

This is the combination of the above two works

## PCB

TODO: the board design files

# Compilation

open the mrgtimer\_firmware project in the arduino ide

this can be done with `make gui` (read the Makefile for instructions)

hopefully i have time to make this work with arduino-cli
