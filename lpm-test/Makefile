CONTIKI_PROJECT = observed monitor sink
#CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
all:$(CONTIKI_PROJECT)

CONTIKI = ..

ifndef TARGET
TARGET=zoul
BOARD=firefly
endif

#MAKE_MAC = MAKE_MAC_TSCH
MAKE_NET = MAKE_NET_NULLNET
include $(CONTIKI)/Makefile.include
