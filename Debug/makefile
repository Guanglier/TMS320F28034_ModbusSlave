################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs2020/ccs/tools/compiler/ti-cgt-c2000_22.6.2.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./DSP2803x_CodeStartBranch.obj" \
"./DSP2803x_DefaultIsr.obj" \
"./DSP2803x_GlobalVariableDefs.obj" \
"./DSP2803x_Gpio.obj" \
"./DSP2803x_PieCtrl.obj" \
"./DSP2803x_PieVect.obj" \
"./DSP2803x_SysCtrl.obj" \
"./DSP2803x_usDelay.obj" \
"./Example_2803xGpioToggle.obj" \
"./app_modbus_test.obj" \
"./drv_modbus_crc.obj" \
"./drv_modbus_slave.obj" \
"./hal_uart.obj" \
"../28034_RAM_lnk.cmd" \
"C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2803x/headers/cmd/DSP2803x_Headers_nonBIOS.cmd" \
$(GEN_CMDS__FLAG) \
-lrts2800_ml.lib \
-lIQmath.lib \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
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
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
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
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
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
Example_2803xGpioToggle.out 

EXE_OUTPUTS__QUOTED += \
"Example_2803xGpioToggle.out" 


# All Target
all: Example_2803xGpioToggle.out

# Tool invocations
Example_2803xGpioToggle.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccs2020/ccs/tools/compiler/ti-cgt-c2000_22.6.2.LTS/bin/cl2000" -v28 -ml -mt --define=_DEBUG --define=LARGE_MODEL -g --diag_suppress=10063 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi -z -m"Example_2803xGpioToggle.map" --stack_size=0x200 --warn_sections -i"C:/ti/ccs2020/ccs/tools/compiler/ti-cgt-c2000_22.6.2.LTS/lib" -i"C:/ti/ccs2020/ccs/tools/compiler/ti-cgt-c2000_22.6.2.LTS/include" -i"C:/Users/user/workspace_ccstheia/Example_2803xGpioToggle" -i"C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2803x/common/lib" -i"C:/ti/c2000/C2000Ware_5_05_00_00/libraries/math/IQmath/c28/lib" --reread_libs --issue_remarks --verbose_diagnostics --xml_link_info="Example_2803xGpioToggle_linkInfo.xml" --entry_point=code_start --rom_model -o "Example_2803xGpioToggle.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "DSP2803x_CodeStartBranch.obj" "DSP2803x_DefaultIsr.obj" "DSP2803x_GlobalVariableDefs.obj" "DSP2803x_Gpio.obj" "DSP2803x_PieCtrl.obj" "DSP2803x_PieVect.obj" "DSP2803x_SysCtrl.obj" "DSP2803x_usDelay.obj" "Example_2803xGpioToggle.obj" "app_modbus_test.obj" "drv_modbus_crc.obj" "drv_modbus_slave.obj" "hal_uart.obj" 
	-$(RM) "DSP2803x_DefaultIsr.d" "DSP2803x_GlobalVariableDefs.d" "DSP2803x_Gpio.d" "DSP2803x_PieCtrl.d" "DSP2803x_PieVect.d" "DSP2803x_SysCtrl.d" "Example_2803xGpioToggle.d" "app_modbus_test.d" "drv_modbus_crc.d" "drv_modbus_slave.d" "hal_uart.d" 
	-$(RM) "DSP2803x_CodeStartBranch.d" "DSP2803x_usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

