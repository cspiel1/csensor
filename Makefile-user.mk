## Local build configuration
## Parameters configured here will override default and ENV values.
## Uncomment and change examples:

#Add your source directories here separated by space
MODULES = app

## ESP_HOME sets the path where ESP tools and SDK are located.
## Windows:
# ESP_HOME = c:/Espressif

## MacOS / Linux:
ESP_HOME=/opt/esp-open-sdk
ESPTOOL2=/opt/esptool2/esptool2
SDK_BASE=/opt/esp-open-sdk/sdk

## SMING_HOME sets the path where Sming framework is located.
## Windows:
# SMING_HOME = c:/tools/sming/Sming

# MacOS / Linux
SMING_HOME=/opt/Sming/Sming

## COM port parameter is reqruied to flash firmware correctly.
## Windows:
# COM_PORT = COM3

# MacOS / Linux:
# COM_PORT = /dev/tty.usbserial

# Com port speed
COM_SPEED	= 115200

# MacOS / Linux:
# COM_PORT = /dev/tty.usbserial
COM_PORT=/dev/ttyUSB0

#### overridable rBoot options ####
## use rboot build mode
RBOOT_ENABLED = 1
## option to completely disable spiffs
DISABLE_SPIFFS  = 1

## enable big flash support (for multiple roms, each in separate 1mb block of flash)
RBOOT_BIG_FLASH ?= 1
## two rom mode (where two roms sit in the same 1mb block of flash)
#RBOOT_TWO_ROMS  ?= 1
## size of the flash chip
# ESP01
# SPI_SIZE        ?= 512K
# RBOOT_SPIFFS_0  ?= 0x60000

#ESP07
# RBOOT_SPIFFS_0  ?= 0x40000
# SPI_SIZE        ?= 1M

# ESP12
SPI_SIZE        ?= 4M
RBOOT_SPIFFS_0  ?= 0x100000

# ESP13
# SPI_SIZE        ?= 16M
# RBOOT_SPIFFS_0  ?= 0x400000

## output file for first rom (.bin will be appended)
#RBOOT_ROM_0     ?= rom0
## input linker file for first rom
#RBOOT_LD_0      ?= rom0.ld
## these next options only needed when using two rom mode
#RBOOT_ROM_1     ?= rom1
#RBOOT_LD_1      ?= rom1.ld
## size of the spiffs to create
SPIFF_SIZE      ?= 65536


## esptool2 path
#ESPTOOL2        ?= esptool2
EXTRA_INCDIR ?= include # default to include if not set by user
EXTRA_INCDIR += $(SMING_HOME)/Libraries/Adafruit_Sensor
