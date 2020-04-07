CONTIKI_PROJECT =  observer-sender observed-sender observed-receiver observer-sink 
all:$(CONTIKI_PROJECT)

CONTIKI = ..

ifndef TARGET
TARGET=zoul
BOARD=firefly
endif

ifneq ($(TARGET),zoul)
$(error Software is platform specific, only for Zolertia Zoul.)
endif

CONTIKI_WITH_IPV6 = 1
MAKE_ROUTING = MAKE_ROUTING_RPL_CLASSIC
include $(CONTIKI)/Makefile.include
