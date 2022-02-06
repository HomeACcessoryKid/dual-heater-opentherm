PROGRAM = main

EXTRA_COMPONENTS = \
	extras/timekeeping \
	extras/http-parser \
	extras/rboot-ota \
    extras/onewire \
    extras/ds18b20 \
    extras/i2s_dma \
    extras/paho_mqtt_c \
	$(abspath esp-wolfssl) \
	$(abspath esp-cjson) \
	$(abspath esp-homekit) \
	$(abspath UDPlogger) \
    
FLASH_SIZE ?= 8
HOMEKIT_SPI_FLASH_BASE_ADDR ?= 0x8C000

EXTRA_CFLAGS += -I../.. -DHOMEKIT_SHORT_APPLE_UUIDS

OT_RECV_PIN?= 4
LED_PIN    ?= 13
SENSOR_PIN ?= 2
SWITCH_PIN ?= 0
EXTRA_CFLAGS += -DOT_RECV_PIN=$(OT_RECV_PIN) -DSENSOR_PIN=$(SENSOR_PIN) -DSWITCH_PIN=$(SWITCH_PIN) -DLED_PIN=$(LED_PIN)

ifdef VERSION
EXTRA_CFLAGS += -DVERSION=\"$(VERSION)\"
endif

EXTRA_CFLAGS += -DUDPLOG_PRINTF_TO_UDP
EXTRA_CFLAGS += -DUDPLOG_PRINTF_ALSO_SERIAL

include $(SDK_PATH)/common.mk

monitor:
	$(FILTEROUTPUT) --port $(ESPPORT) --baud $(ESPBAUD) --elf $(PROGRAM_OUT)

sig:
	openssl sha384 -binary -out firmware/main.bin.sig firmware/main.bin
	printf "%08x" `cat firmware/main.bin | wc -c`| xxd -r -p >>firmware/main.bin.sig
	ls -l firmware
