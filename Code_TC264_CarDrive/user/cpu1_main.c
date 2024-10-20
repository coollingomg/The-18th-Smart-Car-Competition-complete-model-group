#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"


//包含头文件
#include "voltage_sampling/voltage_sampling.h"
#include "zf_driver_pit.h"
#include "timer/timer.h"
#include "motor/motor.h"
#include "key/key.h"
#include "INA226.h"
#include "uart.h"


//-----------------------------------代码区域-----------------------------------
void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断

//-----------------------------------此处编写用户代码 例如外设初始化代码等----------------------------------

    //电压采样初始化
    adc_Init();
    //按键初始化
    my_key_init();
    //与eb通信初始化
    USB_uart_init(eb_using_uart, eb_using_uart_baud, uart_eb_pin_tx, uart_eb_pin_rx);

//-----------------------------------此处编写用户代码 例如外设初始化代码等-----------------------------------

    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {

//-----------------------------------此处编写需要循环执行的代码-----------------------------------

        //电压采样处理
        adc_Handle();
        //数据处理
        USB_Edgeboard_Handle();

//-----------------------------------此处编写需要循环执行的代码-----------------------------------

    }
}

#pragma section all restore
//----------------------------------代码区域----------------------------------
