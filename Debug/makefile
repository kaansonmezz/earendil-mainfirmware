################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include .venv/lib64/python3.14/site-packages/PySide6/Qt/qml/Qt/labs/assetdownloader/objects-RelWithDebInfo/QmlAssetDownloaderPrivateplugin_init/subdir.mk
-include .venv/lib64/python3.14/site-packages/PySide6/Qt/qml/Qt/labs/assetdownloader/objects-RelWithDebInfo/QmlAssetDownloaderPrivate_resources_1/.qt/rcc/subdir.mk
-include .venv/lib/python3.14/site-packages/PySide6/Qt/qml/Qt/labs/assetdownloader/objects-RelWithDebInfo/QmlAssetDownloaderPrivateplugin_init/subdir.mk
-include .venv/lib/python3.14/site-packages/PySide6/Qt/qml/Qt/labs/assetdownloader/objects-RelWithDebInfo/QmlAssetDownloaderPrivate_resources_1/.qt/rcc/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := H7-DMA
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
H7-DMA.elf \

MAP_FILES += \
H7-DMA.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
H7-DMA.list \


# All Target
all: main-build

# Main-build Target
main-build: H7-DMA.elf secondary-outputs

# Tool invocations
H7-DMA.elf H7-DMA.map: $(OBJS) $(USER_OBJS) /home/garth/H7-DMA/STM32H723ZGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "H7-DMA.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"/home/garth/H7-DMA/STM32H723ZGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="H7-DMA.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

H7-DMA.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "H7-DMA.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) H7-DMA.elf H7-DMA.list H7-DMA.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
