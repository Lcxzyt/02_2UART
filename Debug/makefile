################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./ti_msp_dl_config.o" \
"./startup_mspm0g350x_ticlang.o" \
"./User/app/CmdDispatch.o" \
"./User/app/Display.o" \
"./User/app/Heading.o" \
"./User/app/HeadingDrive.o" \
"./User/app/IMU.o" \
"./User/app/IMUTest.o" \
"./User/app/LineFollow.o" \
"./User/app/StreamOutput.o" \
"./User/bsp/Bluetooth.o" \
"./User/bsp/BoardIO.o" \
"./User/bsp/Encoder.o" \
"./User/bsp/MPU6050.o" \
"./User/bsp/Motor.o" \
"./User/bsp/MyI2C.o" \
"./User/bsp/OLED.o" \
"./User/bsp/QMC5883L.o" \
"./User/bsp/Timer.o" \
"./User/bsp/Tracking.o" \
"./User/bsp/uart.o" \
"./User/main/main.o" \
"./User/systool/PID.o" \
"./User/systool/delay.o" \
"./User/task/AutoTrackTask.o" \
"./User/task/TaskController.o" \
"../app_stack.cmd" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \
-Wl,-lapp_stack.cmd \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include User/app/subdir_vars.mk
-include User/bsp/subdir_vars.mk
-include User/main/subdir_vars.mk
-include User/systool/subdir_vars.mk
-include User/task/subdir_vars.mk
-include subdir_rules.mk
-include User/app/subdir_rules.mk
-include User/bsp/subdir_rules.mk
-include User/main/subdir_rules.mk
-include User/systool/subdir_rules.mk
-include User/task/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
02_2UART.out 

EXE_OUTPUTS__QUOTED += \
"02_2UART.out" 


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "02_2UART.out"

# Tool invocations
02_2UART.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Arm Linker - building target: "$@"'
	"D:/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -gdwarf-3 -Wl,-m"02_2UART.map" -Wl,-i"D:/CCS/mspm0_sdk_2_10_00_04/source" -Wl,-i"D:/CCS/WorkSpace/02_2UART" -Wl,-i"D:/CCS/WorkSpace/02_2UART/Debug/syscfg" -Wl,-i"D:/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="02_2UART_linkInfo.xml" -Wl,--rom_model -o "02_2UART.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "ti_msp_dl_config.o" "startup_mspm0g350x_ticlang.o" "User\app\CmdDispatch.o" "User\app\Display.o" "User\app\Heading.o" "User\app\HeadingDrive.o" "User\app\IMU.o" "User\app\IMUTest.o" "User\app\LineFollow.o" "User\app\StreamOutput.o" "User\bsp\Bluetooth.o" "User\bsp\BoardIO.o" "User\bsp\Encoder.o" "User\bsp\MPU6050.o" "User\bsp\Motor.o" "User\bsp\MyI2C.o" "User\bsp\OLED.o" "User\bsp\QMC5883L.o" "User\bsp\Timer.o" "User\bsp\Tracking.o" "User\bsp\uart.o" "User\main\main.o" "User\systool\PID.o" "User\systool\delay.o" "User\task\AutoTrackTask.o" "User\task\TaskController.o" 
	-$(RM) "ti_msp_dl_config.d" "startup_mspm0g350x_ticlang.d" "User\app\CmdDispatch.d" "User\app\Display.d" "User\app\Heading.d" "User\app\HeadingDrive.d" "User\app\IMU.d" "User\app\IMUTest.d" "User\app\LineFollow.d" "User\app\StreamOutput.d" "User\bsp\Bluetooth.d" "User\bsp\BoardIO.d" "User\bsp\Encoder.d" "User\bsp\MPU6050.d" "User\bsp\Motor.d" "User\bsp\MyI2C.d" "User\bsp\OLED.d" "User\bsp\QMC5883L.d" "User\bsp\Timer.d" "User\bsp\Tracking.d" "User\bsp\uart.d" "User\main\main.d" "User\systool\PID.d" "User\systool\delay.d" "User\task\AutoTrackTask.d" "User\task\TaskController.d" 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

