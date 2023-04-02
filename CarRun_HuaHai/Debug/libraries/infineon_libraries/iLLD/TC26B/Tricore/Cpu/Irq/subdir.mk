################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/infineon_libraries/iLLD/TC26B/Tricore/Cpu/Irq/IfxCpu_Irq.c 

OBJS += \
./libraries/infineon_libraries/iLLD/TC26B/Tricore/Cpu/Irq/IfxCpu_Irq.o 

COMPILED_SRCS += \
./libraries/infineon_libraries/iLLD/TC26B/Tricore/Cpu/Irq/IfxCpu_Irq.src 

C_DEPS += \
./libraries/infineon_libraries/iLLD/TC26B/Tricore/Cpu/Irq/IfxCpu_Irq.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/infineon_libraries/iLLD/TC26B/Tricore/Cpu/Irq/%.src: ../libraries/infineon_libraries/iLLD/TC26B/Tricore/Cpu/Irq/%.c libraries/infineon_libraries/iLLD/TC26B/Tricore/Cpu/Irq/subdir.mk
	@echo '正在构建文件： $<'
	@echo '正在调用： TASKING C/C++ Compiler'
	cctc "-fD:/Infineon/demo_CarRun_workspace/CarRun_HuaHai/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O2 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo '已结束构建： $<'
	@echo ' '

libraries/infineon_libraries/iLLD/TC26B/Tricore/Cpu/Irq/%.o: ./libraries/infineon_libraries/iLLD/TC26B/Tricore/Cpu/Irq/%.src libraries/infineon_libraries/iLLD/TC26B/Tricore/Cpu/Irq/subdir.mk
	@echo '正在构建文件： $<'
	@echo '正在调用： TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo '已结束构建： $<'
	@echo ' '


