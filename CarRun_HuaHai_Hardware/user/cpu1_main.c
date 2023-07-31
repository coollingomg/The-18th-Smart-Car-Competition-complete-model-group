#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"



//包含头文件
#include "icm20602_data_pose/icm20602_data_handle.h"
#include "voltage_sampling/voltage_sampling.h"
#include "car_control/car_control.h"
#include "icm20602_data_handle.h"
#include "motor/motor.h"
#include "servo/servo.h"
#include "key/key.h"
#include "uart.h"
#include "pid.h"
#include "Kalman/Kalman_Filter.h"


//-----------------------------------代码区域-----------------------------------
void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断

//-----------------------------------此处编写用户代码 例如外设初始化代码等----------------------------------

    //陀螺仪任务初始化
    icm20602_pose_init();
    //电压采样初始化
    adc_Init();
    //按键初始化
    my_key_init();
    //偏航角加速度滤波，卡尔曼参数初始化
    Kalman_Filter_Init(&kalman_struck1);

//-----------------------------------此处编写用户代码 例如外设初始化代码等-----------------------------------

    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {

//-----------------------------------此处编写需要循环执行的代码-----------------------------------

        //电压采样处理
        adc_Handle();
        //数据处理
        USB_Edgeboard_Handle();
        //姿态角解算处理函数
        icm20602_attitude_Angle_handle();

//-----------------------------------此处编写需要循环执行的代码-----------------------------------

    }
}

#pragma section all restore
//----------------------------------代码区域----------------------------------
