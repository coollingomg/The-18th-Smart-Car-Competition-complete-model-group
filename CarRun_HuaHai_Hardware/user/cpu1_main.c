#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

//包含头文件
#include "voltage_sampling/voltage_sampling.h"
#include "icm20602_data_handle.h"
#include "motor/motor.h"
#include "servo/servo.h"
#include "key/key.h"
#include "uart.h"
#include "pid.h"



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


//-----------------------------------此处编写用户代码 例如外设初始化代码等-----------------------------------

    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {

//-----------------------------------此处编写需要循环执行的代码-----------------------------------

//选择通信模式
#if USING_BLUETOOTH_OR_EGBOARD
//        //舵机转向控制
//        servo_contral(Bluetooth_data.data_angle);
#else
#endif

//        //舵机转向控制线程处理函数
//        SERVO_Handle();
//        //陀螺仪姿态角解算更新
//        icm20602_attitude_Angle_pose(&icm20602_pose, 0.001);
        //电压采样处理
        adc_Handle();

//-----------------------------------此处编写需要循环执行的代码-----------------------------------

    }
}

#pragma section all restore
//----------------------------------代码区域----------------------------------
