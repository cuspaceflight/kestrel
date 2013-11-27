BOARD := uno
ARDUINODIR := $(firstword $(wildcard ~/opt/arduino /usr/share/arduino \
	/Applications/Arduino.app/Contents/Resources/Java \
	$(HOME)/Applications/Arduino.app/Contents/Resources/Java))

# default path to find libraries
LIBRARYPATH ?= ../libraries $(ARDUINODIR)/libraries
include ../arduino.mk