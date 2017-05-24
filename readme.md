# angryRobot compilation instructions:

### host (x86)
AngryRobot $ make host

### target (arduino)
AngryRobot $ export CXXFLAGS_STD=-std=gnu++11
AngryRobot $ export ARDUINO_CORE_PATH=/usr/share/arduino/hardware/arduino/cores/arduino
AngryRobot $ export ARDUINO_VERSION=10801
AngryRobot $ export BOARDS_TXT=/usr/share/arduino/hardware/arduino/boards.txt 
AngryRobot $ export ARDUINO_LIB_PATH=/usr/share/arduino/libraries
AngryRobot $ export ARDUINO_VAR_PATH=/usr/share/arduino/hardware/arduino/variants

AngryRobot $ make target


### working configuration:
[AUTODETECTED]       CURRENT_OS = LINUX  
[AUTODETECTED]       ARDUINO_DIR = /usr/share/arduino  
[COMPUTED]           ARDMK_DIR = /home/quentin/AngryRobot/src (relative to Common.mk)  
[USER]               ARDUINO_VERSION = 10801  
[DEFAULT]            ARCHITECTURE = avr  
[DEFAULT]            ARDMK_VENDOR = arduino  
[AUTODETECTED]       ARDUINO_PREFERENCES_PATH = /home/quentin/.arduino15/preferences.txt  
[AUTODETECTED]       ARDUINO_SKETCHBOOK = /home/quentin/Arduino (from arduino preferences file)  
[BUNDLED]            AVR_TOOLS_DIR = /usr/share/arduino/hardware/tools/avr (in Arduino distribution)  
[USER]               ARDUINO_LIB_PATH = /usr/share/arduino/libraries  
[COMPUTED]           ARDUINO_PLATFORM_LIB_PATH = /usr/share/arduino/hardware/arduino/avr/libraries (from ARDUINO_DIR)  
[USER]               ARDUINO_VAR_PATH = /usr/share/arduino/hardware/arduino/variants   
[USER]               BOARDS_TXT = /usr/share/arduino/hardware/arduino/boards.txt  
[DEFAULT]            USER_LIB_PATH = /home/quentin/Arduino/libraries (in user sketchbook)  
[DEFAULT]            PRE_BUILD_HOOK = pre-build-hook.sh   
[DEFAULT]            BOARD_TAG = uno  
[COMPUTED]           CORE = arduino (from build.core)  
[COMPUTED]           VARIANT = standard (from build.variant)  
[COMPUTED]           OBJDIR = build-uno (from BOARD_TAG)  
[USER]               ARDUINO_CORE_PATH = /usr/share/arduino/hardware/arduino/cores/arduino   
                     No .pde or .ino files found. If you are compiling .c or .cpp files then you need to explicitly
include Arduino header files   
[ASSUMED]            MONITOR_BAUDRATE = 9600   
[DEFAULT]            OPTIMIZATION_LEVEL = s   
[DEFAULT]            MCU_FLAG_NAME = mmcu   
[DEFAULT]            CFLAGS_STD =    
[USER]               CXXFLAGS_STD = -std=gnu++11   
[AUTODETECTED]       DEVICE_PATH =   
[DEFAULT]            FORCE_MONITOR_PORT =   
[AUTODETECTED]       Size utility: AVR-aware for enhanced output  
[COMPUTED]           BOOTLOADER_PARENT = /usr/share/arduino/hardware/arduino/avr/bootloaders (from ARDUINO_DIR)  
[COMPUTED]           ARDMK_VERSION = 1.5  
[COMPUTED]           CC_VERSION = 4.8.2 (avr-gcc)  


# angryRobot test instructions:
TODO

# arduino useful info
![alt text](https://i.stack.imgur.com/dVkQU.jpg)
