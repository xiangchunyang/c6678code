################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
main.obj: ../main.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/ti/ccsv5/tools/compiler/c6000_7.4.1/bin/cl6x" -mv6600 -g --include_path="C:/ti/ccsv5/tools/compiler/c6000_7.4.1/include" --include_path="F:/CHIP/C6678/C6678_Lib/PublishEdition/1701_1706/TI_c6678cbb_Dri_v1.18/1_Program/TI_c6678cbb_Dri_v1.18/2_Lib_Ref/3_Demo/little_endian/main_vpx/send" --include_path="C:/ti/pdk_C6678_1_1_2_5/packages" --include_path="C:/ti/pdk_C6678_1_1_2_5/packages/ti/csl/src/intc" --include_path="C:/ti/pdk_C6678_1_1_2_5/packages/ti/platform/evmc6678l/platform_lib/include" --include_path="C:/ti/pdk_C6678_1_1_2_5/packages/ti/csl" --display_error_number --diag_warning=225 --abi=eabi --preproc_with_compile --preproc_dependency="main.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

spi_norflash.obj: ../spi_norflash.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/ti/ccsv5/tools/compiler/c6000_7.4.1/bin/cl6x" -mv6600 -g --include_path="C:/ti/ccsv5/tools/compiler/c6000_7.4.1/include" --include_path="F:/CHIP/C6678/C6678_Lib/PublishEdition/1701_1706/TI_c6678cbb_Dri_v1.18/1_Program/TI_c6678cbb_Dri_v1.18/2_Lib_Ref/3_Demo/little_endian/main_vpx/send" --include_path="C:/ti/pdk_C6678_1_1_2_5/packages" --include_path="C:/ti/pdk_C6678_1_1_2_5/packages/ti/csl/src/intc" --include_path="C:/ti/pdk_C6678_1_1_2_5/packages/ti/platform/evmc6678l/platform_lib/include" --include_path="C:/ti/pdk_C6678_1_1_2_5/packages/ti/csl" --display_error_number --diag_warning=225 --abi=eabi --preproc_with_compile --preproc_dependency="spi_norflash.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


