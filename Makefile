####################################
# LOCAL PARAMETERS
# you can edit these,
# or override on the command line
# with make PARAM=VAL
####################################

# set this to your installation dir
ARDUINO_INSTALL_DIR ?= ~/Software/arduino-1.8.19

######################
# DERIVED PARAMETERS
# don't edit these
######################

# TODO use arduino cli... arduino-builder sucks
# the gui is also not great but whatever

ARDUINO_GUI := $(ARDUINO_INSTALL_DIR)/arduino
.PHONY: gui
gui:
	$(ARDUINO_GUI) mrgtimer_firmware/mrgtimer_firmware.ino 
