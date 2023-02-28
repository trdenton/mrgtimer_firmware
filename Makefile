####################################
# LOCAL PARAMETERS
# you can edit these,
# or override on the command line
# with make PARAM=VAL
####################################

# set this to your installation dir
ARDUINO_INSTALL_DIR ?= ~/Software/arduino-1.8.19
ARDUINO_CLI ?= ~/Software/arduino-cli

######################
# DERIVED PARAMETERS
# don't edit these
######################

ACLI := $(ARDUINO_CLI)

# if it's in the path, use that one
ifneq ($(shell which arduino-cli),)
ACLI := $(shell which arduino-cli)
endif

ARDUINO_GUI := $(ARDUINO_INSTALL_DIR)/arduino

.PHONY: verify
verify:
	$(ACLI) compile --fqbn arduino:avr:nano mrgtimer_firmware

.PHONY: upload
upload:
	$(ACLI) upload -P avrispmkii --fqbn arduino:avr:nano mrgtimer_firmware

.PHONY: gui
gui:
	$(ARDUINO_GUI) mrgtimer_firmware/mrgtimer_firmware.ino


# youll want to go to config > auto crlf
.PHONY: term 
term:
	gtkterm -e --port /dev/ttyUSB0 --speed $(shell grep Serial.begin mrgtimer_firmware/mrgtimer_firmware.ino | sed -e 's/.*(//;s/).*//;') &
