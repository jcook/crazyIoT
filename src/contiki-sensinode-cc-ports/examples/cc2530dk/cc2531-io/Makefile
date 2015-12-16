# To build for the cc2531 USB stick:
DEFINES+=MODEL_CC2531=1,

DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
DEFINES+=BUTTON_SENSOR_ON=1

# We need uIPv6, therefore we also need banking
HAVE_BANKING=1
UIP_CONF_IPV6=1
UIP_CONF_RPL=1

CONTIKI_PROJECT = cc2531-io

all: $(CONTIKI_PROJECT) 

CONTIKI = ../../..

include $(CONTIKI)/Makefile.include
