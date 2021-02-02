CONTIKI_PROJECT =  observed-sender observed-receiver energest-sink
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
all:$(CONTIKI_PROJECT)

CONTIKI = ../..

ifndef TARGET
TARGET=zoul
BOARD=firefly
endif

ifneq ($(TARGET),zoul)
$(error Software is platform specific, only for Zolertia Zoul.)
endif

MAKE_MAC = MAKE_MAC_TSCH
MAKE_WITH_ORCHESTRA ?= 0
#MAKE_MAC ?= MAKE_MAC_CSMA
MAKE_NET = MAKE_NET_NULLNET
include $(CONTIKI)/Makefile.include
