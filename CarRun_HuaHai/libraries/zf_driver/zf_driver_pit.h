#ifndef _zf_driver_pit_h_
#define _zf_driver_pit_h_

#include "zf_common_typedef.h"

typedef enum // 枚举通道号
{
    CCU60_CH0,
    CCU60_CH1,
    CCU61_CH0,
    CCU61_CH1,
}pit_index_enum;


#define pit_clear_flag(pit_index) (IfxCcu6_clearInterruptStatusFlag(IfxCcu6_getAddress((IfxCcu6_Index)(pit_index / 2)), (IfxCcu6_InterruptSource)(7+((pit_index % 2)*2))))

//====================================================PIT 基础函数====================================================
void pit_close               (pit_index_enum pit_index);
void pit_start               (pit_index_enum pit_index);
void pit_all_close           (void);
void pit_disable             (pit_index_enum pit_index);
void pit_enable              (pit_index_enum pit_index);
void pit_init                (pit_index_enum pit_index, uint32 time);
//====================================================PIT 基础函数====================================================

//====================================================PIT 扩展函数====================================================
//-------------------------------------------------------------------------------------------------------------------
//  函数简介      pit_ms初始化
//  参数说明      pit_index           选择CCU6模块
//  参数说明      time                周期时间(单位：毫秒)
//  返回参数      void
//  使用示例      pit_ms_init(CCU60_CH0, 5);      // 设置周期中断5ms
//-------------------------------------------------------------------------------------------------------------------
#define pit_ms_init(pit_index, time)  pit_init((pit_index), (time*1000))   // (单位为 毫秒)
//-------------------------------------------------------------------------------------------------------------------
//  函数简介      pit_us初始化
//  参数说明      pit_index           选择CCU6模块
//  参数说明      time                周期时间(单位：微秒)
//  返回参数      void
//  使用示例      pit_us_init(CCU60_CH0, 5);      // 设置周期中断5us
//-------------------------------------------------------------------------------------------------------------------
#define pit_us_init(pit_index, time)  pit_init((pit_index), (time))        // (单位为 微秒)
//====================================================PIT 扩展函数====================================================

#endif
