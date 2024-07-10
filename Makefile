# The name of your project (used to name the compiled .hex file)
TARGET = main
TEENSY = 36
# Set to 24000000, 48000000, or 96000000 to set CPU core speed
TEENSY_CORE_SPEED = 180000000
CPUARCH = cortex-m4

ARDUINOPATH = /Applications/Arduino.app/Contents/Java

# LIBRARYPATH = $(abspath $(ARDUINOPATH)/libraries)
LIBRARYPATH = $(abspath libraries)

COMPILERPATH = $(abspath $(ARDUINOPATH)/hardware/tools/arm/bin)
# COMPILERPATH = $(abspath $(ARDUINOPATH)/hardware/tools/arm/bin)

# path location for Teensy Loader, teensy_post_compile and teensy_reboot (on Linux)
TOOLSPATH = $(abspath $(ARDUINOPATH)/hardware/tools)

# path location for Teensy 3 core
# COREPATH = $(abspath $(ARDUINOPATH)/hardware/teensy/avr/cores/teensy3)
COREPATH = $(abspath core)

# path location for the arm-none-eabi compiler
COMPILERPATH = $(TOOLSPATH)/arm/bin

BUILDDIR = $(abspath $(CURDIR)/build)


ARDUINO = 10809

MCU=MK66FX1M0
# make it lower case
LOWER_MCU := $(subst A,a,$(subst B,b,$(subst C,c,$(subst D,d,$(subst E,e,$(subst F,f,$(subst G,g,$(subst H,h,$(subst I,i,$(subst J,j,$(subst K,k,$(subst L,l,$(subst M,m,$(subst N,n,$(subst O,o,$(subst P,p,$(subst Q,q,$(subst R,r,$(subst S,s,$(subst T,t,$(subst U,u,$(subst V,v,$(subst W,w,$(subst X,x,$(subst Y,y,$(subst Z,z,$(MCU)))))))))))))))))))))))))))
MCU_LD = $(LOWER_MCU).ld

# configurable options
OPTIONS = -DUSB_SERIAL -DLAYOUT_US_ENGLISH
OPTIONS += -D__$(MCU)__ -DARDUINO=10809 -DTEENSYDUINO=144
#OPTIONS += -DUSING_MAKEFILE


# CPPFLAGS = compiler options for C and C++
CPPFLAGS = -g -Os -mthumb -ffunction-sections -fdata-sections -nostdlib -MMD $(OPTIONS) -DTEENSYDUINO=144 -DF_CPU=$(TEENSY_CORE_SPEED) -Isrc -I$(COREPATH)
# CPPFLAGS = -Wall -g -Os -mthumb -ffunction-sections -fdata-sections -nostdlib -MMD $(OPTIONS) -DTEENSYDUINO=144 -DF_CPU=$(TEENSY_CORE_SPEED) -Isrc -I$(COREPATH)

# compiler options for C++ only
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti

# compiler options for C only
CFLAGS =

# linker options
#LDFLAGS = -Os -Wl,--gc-sections -mthumb
LDFLAGS = -Os -Wl,--gc-sections,--defsym=__rtc_localtime=0 --specs=nano.specs -mthumb -mcpu=cortex-m4

# linker options
#LDFLAGS = -Os -Wl,--gc-sections,--defsym=__rtc_localtime=0 

# additional libraries to link
LIBS = -lm

# compiler options specific to teensy version
ifeq ($(TEENSY), 30)
    CPPFLAGS += -D__MK20DX128__ -mcpu=cortex-m4
    LDSCRIPT = $(COREPATH)/mk20dx128.ld
    LDFLAGS += -mcpu=cortex-m4 -T$(LDSCRIPT)
else ifeq ($(TEENSY), 31)
    CPPFLAGS += -D__MK20DX256__ -mcpu=cortex-m4
    LDSCRIPT = $(COREPATH)/mk20dx256.ld
    LDFLAGS += -mcpu=cortex-m4 -T$(LDSCRIPT)
else ifeq ($(TEENSY), LC)
    CPPFLAGS += -D__MKL26Z64__ -mcpu=cortex-m0plus
    LDSCRIPT = $(COREPATH)/mkl26z64.ld
    LDFLAGS += -mcpu=cortex-m0plus -T$(LDSCRIPT)
    LIBS += -larm_cortexM0l_math
else ifeq ($(TEENSY), 35)
    CPPFLAGS += -D__MK64FX512__ -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
    LDSCRIPT = $(COREPATH)/mk64fx512.ld
    LDFLAGS += -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T$(LDSCRIPT)
    LIBS += -larm_cortexM4lf_math
else ifeq ($(TEENSY), 36)
    CPPFLAGS += -D__MK66FX1M0__ -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
    LDSCRIPT = $(COREPATH)/mk66fx1m0.ld
    LDFLAGS += -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T$(LDSCRIPT)
    LIBS += -larm_cortexM4lf_math
else
    $(error Invalid setting for TEENSY)
endif

# set arduino define if given
ifdef ARDUINO
	CPPFLAGS += -DARDUINO=$(ARDUINO)
else
	CPPFLAGS += -DUSING_MAKEFILE
endif

# names for the compiler programs
CC = $(abspath $(COMPILERPATH))/arm-none-eabi-gcc
CXX = $(abspath $(COMPILERPATH))/arm-none-eabi-g++
OBJCOPY = $(abspath $(COMPILERPATH))/arm-none-eabi-objcopy
SIZE = $(abspath $(COMPILERPATH))/arm-none-eabi-size

# automatically create lists of the sources and objects
LC_FILES := $(wildcard $(LIBRARYPATH)/*/*.c) $(wildcard $(LIBRARYPATH)/*/*/*.c) 
LCPP_FILES := $(wildcard $(LIBRARYPATH)/*/*.cpp)  $(wildcard $(LIBRARYPATH)/*/*/*.cpp) 
# $(wildcard $(LIBRARYPATH)/*/*/*/*.cpp)
TC_FILES := $(wildcard $(COREPATH)/*.c)
TCPP_FILES := $(wildcard $(COREPATH)/*.cpp)
C_FILES := $(wildcard src/*.c)
CPP_FILES := $(wildcard src/*.cpp)
INO_FILES := $(wildcard *.ino)

USER_LIBS = $(HOME)/Documents/Arduino/libraries
#$(foreach lib,$(filter %/, $(wildcard $(USER_LIBS)/*/)), -I$(lib))

# include paths for libraries
L_INC := $(foreach lib,$(filter %/, $(wildcard $(LIBRARYPATH)/*/)), -I$(lib))  $(foreach lib,$(filter %/, $(wildcard $(LIBRARYPATH)/*/*/)), -I$(lib)) 

SOURCES := $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o) $(INO_FILES:.ino=.o) $(TC_FILES:.c=.o) $(TCPP_FILES:.cpp=.o) $(LC_FILES:.c=.o) $(LCPP_FILES:.cpp=.o)
# SOURCES := $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o) $(INO_FILES:.ino=.o) $(TC_FILES:.c=.o) $(TCPP_FILES:.cpp=.o) 
OBJS := $(foreach src,$(SOURCES), $(BUILDDIR)/$(src))

all: hex

build: $(TARGET).elf

hex: $(TARGET).hex

post_compile: $(TARGET).hex
	@$(abspath $(TOOLSPATH))/teensy_post_compile -file="$(basename $<)" -path=$(CURDIR) -tools="$(abspath $(TOOLSPATH))"

reboot:
	@-$(abspath $(TOOLSPATH))/teensy_reboot

upload: post_compile reboot

$(BUILDDIR)/%.o: %.c
	@echo -e "[CC]\t$<"
	@mkdir -p "$(dir $@)"
	@$(CC) $(CPPFLAGS) $(CFLAGS) $(L_INC) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.cpp
	@echo -e "[CXX]\t$<"
	@mkdir -p "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(L_INC) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.ino
	@echo -e "[CXX]\t$<"
	@mkdir -p "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(L_INC) -o "$@" -x c++ -include Arduino.h -c "$<"

$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@echo -e "[LD]\t$@"
	@$(CC) $(LDFLAGS) -o "$@" $(OBJS) $(LIBS)

%.hex: %.elf
	@echo -e "[HEX]\t$@"
	@$(SIZE) "$<"
	@$(OBJCOPY) -O ihex -R .eeprom "$<" "$@"

# compiler generated dependency info
-include $(OBJS:.o=.d)

clean:
	@echo Cleaning...
	@rm -rf "$(BUILDDIR)"
	@rm -f "$(TARGET).elf" "$(TARGET).hex"

local: post_compile
	./teensy_loader_cli --mcu=mk66fx1m0 -w -v main.hex

remote: post_compile
	scp main.hex pi@raspberrypi.local:/home/pi/code.hex
	ssh -t pi@raspberrypi.local "sleep 1 && python prog.py && sleep 1" &
	ssh -t pi@raspberrypi.local "./teensy_loader_cli/teensy_loader_cli --mcu=TEENSY$(TEENSY) -w -v code.hex && sleep 1"

remote-reset:
	ssh -t pi@raspberrypi.local "sleep 1 && python reset.py && sleep 1"

remote-prog:
	ssh -t pi@raspberrypi.local "sleep 1 && python prog.py && sleep 1"

remote-monitor:
	ssh -t pi@raspberrypi.local "python monitor.py"

remote-reboot:
	ssh -t pi@raspberrypi.local "sudo reboot"
