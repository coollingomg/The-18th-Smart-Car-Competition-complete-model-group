#include "zf_common_headfile.h"
#include "isr_config.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中
// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 interrupt_global_enable(0); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 interrupt_global_disable(); 来拒绝响应任何的中断，因此需要我们自己手动调用 interrupt_global_enable(0); 来开启中断的响应。


//包含头文件
#include "Kalman/Kalman_Filter.h"
#include "motor/motor.h"
#include "servo/servo.h"
#include "timer/timer.h"
#include "key/key.h"
#include "my_flash/my_flash.h"
#include "Buzzer/buzzer.h"
#include "car_control/car_control.h"
#include "pid.h"
#include "uart.h"



//----------------------------------代码区域----------------------------------
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口

//----------------------------------此处编写用户代码 例如外设初始化代码等----------------------------------

    //加载flash区的存储数据
    my_flash_init();
    //先初始化舵机，解决在中断中控制舵机卡死的bug
    servo_init();
    //电机初始化
    motor_init();
     //线程初始化
    timer_Init();
    //蜂鸣器初始化
    Buzzer_Init();
    //与eb通信初始化
    USB_uart_init(eb_using_uart, eb_using_uart_baud, uart_eb_pin_tx, uart_eb_pin_rx);
    //通信连接提示灯初始化
    gpio_init(P20_9, GPO, 1, GPO_PUSH_PULL);
    //初始化完成后，拉高电平唤醒电机驱动板
    gpio_init(P21_3, GPO, 1, GPO_PUSH_PULL);
    //初始化引脚，用于提示数据接受是否错误
    gpio_init(P21_5, GPO, 1, GPO_PUSH_PULL);
    //卡尔曼参数初始化，电机返回速度滤波
    Kalman_Filter_Init(&kalman_struck);
    //智能车控制参数初始化
    ICAR_Init();
    //初始化完成，蜂鸣器提示音
    Buzzer_Enable(BuzzerSysStart);

//----------------------------------此处编写用户代码 例如外设初始化代码等----------------------------------

    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE)
    {

//----------------------------------此处编写需要循环执行的代码----------------------------------

        //数据处理
        USB_Edgeboard_Handle();
        //flash任务处理
        my_flash_Handle();
        //蜂鸣器控制
        Buzzer_Handle();
        //智能车控制
        ICAR_Handle();

//----------------------------------此处编写需要循环执行的代码----------------------------------

    }
}

#pragma section all restore
//----------------------------------代码区域----------------------------------

