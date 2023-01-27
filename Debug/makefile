################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Middlewares/Third_Party/FatFs/src/option/subdir.mk
-include Middlewares/Third_Party/FatFs/src/subdir.mk
-include FATFS/Target/subdir.mk
-include FATFS/App/subdir.mk
-include Drivers/STM32L1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := Budilnik_FreeRTOS+RTC+USART_stm32l152re_project
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Budilnik_FreeRTOS+RTC+USART_stm32l152re_project.elf \

MAP_FILES += \
Budilnik_FreeRTOS+RTC+USART_stm32l152re_project.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Budilnik_FreeRTOS+RTC+USART_stm32l152re_project.list \


# All Target
all: main-build

# Main-build Target
main-build: Budilnik_FreeRTOS+RTC+USART_stm32l152re_project.elf secondary-outputs

# Tool invocations
Budilnik_FreeRTOS+RTC+USART_stm32l152re_project.elf Budilnik_FreeRTOS+RTC+USART_stm32l152re_project.map: $(OBJS) $(USER_OBJS) C:\Users\telar\Desktop\Udemy\ Courses\Budilnik-3_FreeRTOS+RTC+USART+Button+Interrupt+FatFS+SPI+TIM+DMA+DAC_stm32l152re_project\STM32L152RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Budilnik_FreeRTOS+RTC+USART_stm32l152re_project.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\telar\Desktop\Udemy Courses\Budilnik-3_FreeRTOS+RTC+USART+Button+Interrupt+FatFS+SPI+TIM+DMA+DAC_stm32l152re_project\STM32L152RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Budilnik_FreeRTOS+RTC+USART_stm32l152re_project.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Budilnik_FreeRTOS+RTC+USART_stm32l152re_project.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Budilnik_FreeRTOS+RTC+USART_stm32l152re_project.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Budilnik_FreeRTOS+RTC+USART_stm32l152re_project.elf Budilnik_FreeRTOS+RTC+USART_stm32l152re_project.list Budilnik_FreeRTOS+RTC+USART_stm32l152re_project.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
