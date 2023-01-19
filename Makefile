# Имя программы и собранного бинарника
TARGET = start-stop-tinyx14

# путь к каталогу с GCC
AVRCCDIR = /usr/bin/

#само название компилятора, мало ли, вдруг оно когда-нибудь поменяется
CC = avr-gcc
OBJCOPY = avr-objcopy
SIZE = avr-size

# каталог в который будет осуществляться сборка, что бы не засерать остальные каталоги
BUILD_DIR = build

# название контроллера для компилятора
MCU = attiny414

#флаги для компилятора
OPT = -Os
C_FLAGS = -mmcu=$(MCU) $(OPT) -Wall

# параметры для AVRDUDE
DUDE_MCU = t414
PORT = /dev/ttyUSB0
PORTSPEED = 115200

# DEFINы
DEFINES = \
-D__AVR_ATtiny414__ \
-DF_CPU=20000000UL\
-DTACHO=1

# пути к заголовочным файлам
C_INCLUDES =  \
-I/usr/lib/avr/include

# файлы программы
C_SOURCES = \
	src/main.cpp

# служебные переменные
OBJ_FILES = $(C_SOURCES:.cpp=.o)
ASM_FILES = $(C_SOURCES:.cpp=.s)
OUT_OBJ = $(addprefix $(BUILD_DIR)/, $(notdir $(OBJ_FILES)))

# правила для сборки

all: $(TARGET).hex

$(TARGET).hex: $(TARGET).elf
	$(AVRCCDIR)$(OBJCOPY) -j .text -j .data -O ihex $(BUILD_DIR)/$< $(BUILD_DIR)/$@
	$(AVRCCDIR)$(SIZE) -C --format=avr --mcu=$(MCU) $(BUILD_DIR)/$<

$(TARGET).elf: $(OBJ_FILES) $(ASM_FILES)
	mkdir -p $(BUILD_DIR)
	$(AVRCCDIR)$(CC) $(C_FLAGS) $(DEFINES) $(OUT_OBJ) -o $(BUILD_DIR)/$@

%.o: %.cpp
	echo $^
	$(AVRCCDIR)$(CC) -c $(C_FLAGS) $(DEFINES) $(C_INCLUDES) $< -o $(BUILD_DIR)/$(@F)

%.s: %.cpp
	echo $^
	$(AVRCCDIR)$(CC) -S -g3 $(C_FLAGS) $(DEFINES) $(C_INCLUDES) $< -o $(BUILD_DIR)/$(@F)

clean:
	rm -f $(BUILD_DIR)/*

prog: $(TARGET).hex
	avrdude -p $(DUDE_MCU) -c serialupdi -P $(PORT) -b $(PORTSPEED) -U flash:w:$(BUILD_DIR)/$(TARGET).hex