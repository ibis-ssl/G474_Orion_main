# G474 Main最小ブートローダーをSTM32CubeCLTのGNU toolchainで再現ビルドする。

BUILD_DIR := Build
TARGET := $(BUILD_DIR)/G474_Orion_bootloader

CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE := arm-none-eabi-size

CPU_FLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS := $(CPU_FLAGS) -std=gnu11 -DSTM32G474xx -O2 -g3 -Wall -Wextra -Werror -ffunction-sections -fdata-sections -fstack-usage
CFLAGS += -IInc -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include
LDFLAGS := $(CPU_FLAGS) -TSTM32G474RETX_BOOT.ld -nostartfiles --specs=nano.specs --specs=nosys.specs -Wl,-Map=$(TARGET).map -Wl,--gc-sections -Wl,--print-memory-usage

OBJS := \
	$(BUILD_DIR)/startup.o \
	$(BUILD_DIR)/main.o \
	$(BUILD_DIR)/system_boot.o \
	$(BUILD_DIR)/board_io.o \
	$(BUILD_DIR)/boot_crc32c.o \
	$(BUILD_DIR)/boot_image.o

.PHONY: all clean

all: $(TARGET).elf $(TARGET).bin $(TARGET).hex $(TARGET).list
	$(SIZE) $(TARGET).elf

$(BUILD_DIR):
	@if not exist "$(BUILD_DIR)" mkdir "$(BUILD_DIR)"

$(BUILD_DIR)/startup.o: ../Core/Startup/startup_stm32g474retx.s | $(BUILD_DIR)
	$(CC) $(CPU_FLAGS) -x assembler-with-cpp -c $< -o $@

$(BUILD_DIR)/%.o: Src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJS) STM32G474RETX_BOOT.ld
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(TARGET).list: $(TARGET).elf
	$(OBJDUMP) -h -S $< > $@

clean:
	rm -rf $(BUILD_DIR)
