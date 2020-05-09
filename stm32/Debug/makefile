################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/u8glib/src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
PreHeater_Control.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
PreHeater_Control.list \

OBJCOPY_BIN += \
PreHeater_Control.bin \


# All Target
all: PreHeater_Control.elf secondary-outputs

# Tool invocations
PreHeater_Control.elf: $(OBJS) $(USER_OBJS) /home/cronos/stm32/PreHeater_Control/STM32F030K6TX_FLASH.ld
	arm-none-eabi-gcc -o "PreHeater_Control.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"/home/cronos/stm32/PreHeater_Control/STM32F030K6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="PreHeater_Control.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

PreHeater_Control.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "PreHeater_Control.list"
	@echo 'Finished building: $@'
	@echo ' '

PreHeater_Control.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "PreHeater_Control.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
