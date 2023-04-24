################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
mqtt/%.obj: ../mqtt/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/Program Files/ti/ccs1220/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="D:/Program Files/ti/ccs1220/ccs/ccs_base/arm/include" --include_path="D:/Program Files/ti/ccs1220/ccs/ccs_base/arm/include/CMSIS" --include_path="C:/Users/Raheel Farouk/Documents/!University Work/2023 Spring/ECE1188/Workspace/GrandPrixDashboard-MQTT" --include_path="C:/Users/Raheel Farouk/Documents/!University Work/2023 Spring/ECE1188/Workspace/GrandPrixDashboard-MQTT" --include_path="C:/Users/Raheel Farouk/Documents/!University Work/2023 Spring/ECE1188/Workspace/GrandPrixDashboard-MQTT/simplelink/include" --include_path="C:/Users/Raheel Farouk/Documents/!University Work/2023 Spring/ECE1188/Workspace/GrandPrixDashboard-MQTT/simplelink/source" --include_path="C:/Users/Raheel Farouk/Documents/!University Work/2023 Spring/ECE1188/Workspace/GrandPrixDashboard-MQTT/board" --include_path="C:/Users/Raheel Farouk/Documents/!University Work/2023 Spring/ECE1188/Workspace/GrandPrixDashboard-MQTT/cli_uart" --include_path="C:/Users/Raheel Farouk/Documents/!University Work/2023 Spring/ECE1188/Workspace/GrandPrixDashboard-MQTT/driverlib/MSP432P4xx" --include_path="C:/Users/Raheel Farouk/Documents/!University Work/2023 Spring/ECE1188/Workspace/GrandPrixDashboard-MQTT/spi_cc3100" --include_path="C:/Users/Raheel Farouk/Documents/!University Work/2023 Spring/ECE1188/Workspace/GrandPrixDashboard-MQTT/uart_cc3100" --include_path="C:/Users/Raheel Farouk/Documents/!University Work/2023 Spring/ECE1188/Workspace/GrandPrixDashboard-MQTT/mqtt" --include_path="D:/Program Files/ti/ccs1220/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs --define=_USE_CLI_ -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="mqtt/$(basename $(<F)).d_raw" --obj_directory="mqtt" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


