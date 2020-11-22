PROGRAM = main

EXTRA_COMPONENTS = \
	extras/http-parser \
	extras/rboot-ota \
    extras/onewire \
    extras/ds18b20 \
	$(abspath esp-wolfssl) \
	$(abspath esp-cjson) \
	$(abspath esp-homekit) \
	$(abspath UDPlogger) \

#    $(abspath esp-adv-button)
    
FLASH_SIZE ?= 8
HOMEKIT_SPI_FLASH_BASE_ADDR ?= 0x8C000

EXTRA_CFLAGS += -I../.. -DHOMEKIT_SHORT_APPLE_UUIDS

RELAY_PIN  ?= 12
LED_PIN    ?= 13
SENSOR_PIN ?= 2
EXTRA_CFLAGS += -DRELAY_PIN=$(RELAY_PIN) -DSENSOR_PIN=$(SENSOR_PIN) -DLED_PIN=$(LED_PIN)
SETPOINT   ?= 27.0F
HYSTERESIS ?= 1.0F
EXTRA_CFLAGS += -DHYSTERESIS=$(HYSTERESIS) -DSETPOINT=$(SETPOINT)

ifdef VERSION
EXTRA_CFLAGS += -DVERSION=\"$(VERSION)\"
endif

EXTRA_CFLAGS += -DUDPLOG_PRINTF_TO_UDP
EXTRA_CFLAGS += -DUDPLOG_PRINTF_ALSO_SERIAL

include $(SDK_PATH)/common.mk

monitor:
	$(FILTEROUTPUT) --port $(ESPPORT) --baud $(ESPBAUD) --elf $(PROGRAM_OUT)

signature:
	$(openssl sha384 -binary -out firmware/main.bin.sig firmware/main.bin)
	$(printf "%08x" `cat firmware/main.bin | wc -c`| xxd -r -p >>firmware/main.bin.sig)
