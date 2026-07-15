################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-1357015024: ../empty.syscfg
	@echo 'SysConfig - building file: "$<"'
	"D:/CCS/sysconfig_1.26.2/sysconfig_cli.bat" -s "D:/CCS/mspm0_sdk_2_10_00_04/.metadata/product.json" --script "D:/CCS/WorkSpace/02_2UART/empty.syscfg" -o "." --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-1357015024 ../empty.syscfg
device.opt: build-1357015024
device.cmd.genlibs: build-1357015024
ti_msp_dl_config.c: build-1357015024
ti_msp_dl_config.h: build-1357015024
Event.dot: build-1357015024

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/CCS/WorkSpace/02_2UART" -I"D:/CCS/WorkSpace/02_2UART/Debug" -I"D:/CCS/WorkSpace/02_2UART/User/main" -I"D:/CCS/WorkSpace/02_2UART/User/systool" -I"D:/CCS/WorkSpace/02_2UART/User/task" -I"D:/CCS/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/CCS/mspm0_sdk_2_10_00_04/source" -I"D:/CCS/WorkSpace/02_2UART/User/bsp" -I"D:/CCS/WorkSpace/02_2UART/User/app" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: D:/CCS/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/CCS/WorkSpace/02_2UART" -I"D:/CCS/WorkSpace/02_2UART/Debug" -I"D:/CCS/WorkSpace/02_2UART/User/main" -I"D:/CCS/WorkSpace/02_2UART/User/systool" -I"D:/CCS/WorkSpace/02_2UART/User/task" -I"D:/CCS/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/CCS/mspm0_sdk_2_10_00_04/source" -I"D:/CCS/WorkSpace/02_2UART/User/bsp" -I"D:/CCS/WorkSpace/02_2UART/User/app" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


