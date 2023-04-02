################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/icm20602_data_pose/icm20602_data_handle.c \
../code/icm20602_data_pose/math_my.c \
../code/icm20602_data_pose/pose.c 

OBJS += \
./code/icm20602_data_pose/icm20602_data_handle.o \
./code/icm20602_data_pose/math_my.o \
./code/icm20602_data_pose/pose.o 

COMPILED_SRCS += \
./code/icm20602_data_pose/icm20602_data_handle.src \
./code/icm20602_data_pose/math_my.src \
./code/icm20602_data_pose/pose.src 

C_DEPS += \
./code/icm20602_data_pose/icm20602_data_handle.d \
./code/icm20602_data_pose/math_my.d \
./code/icm20602_data_pose/pose.d 


# Each subdirectory must supply rules for building sources it contributes
code/icm20602_data_pose/%.src: ../code/icm20602_data_pose/%.c code/icm20602_data_pose/subdir.mk
	@echo '正在构建文件： $<'
	@echo '正在调用： TASKING C/C++ Compiler'
	cctc "-fD:/Infineon/demo_CarRun_workspace/CarRun_HuaHai/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O2 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo '已结束构建： $<'
	@echo ' '

code/icm20602_data_pose/%.o: ./code/icm20602_data_pose/%.src code/icm20602_data_pose/subdir.mk
	@echo '正在构建文件： $<'
	@echo '正在调用： TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo '已结束构建： $<'
	@echo ' '


