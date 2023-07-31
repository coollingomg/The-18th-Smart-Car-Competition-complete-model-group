#ifndef _zf_driver_delay_h_
#define _zf_driver_delay_h_

#include "zf_common_typedef.h"


void    system_delay                (uint32 time, uint32 num);

//====================================================延时 基础函数====================================================
#define system_delay_ms(time)       system_delay((1000000), (time))    // 设置延时时间  单位ms
#define system_delay_us(time)       system_delay((time*1000), (1))     // 设置延时时间  单位us
#define system_delay_ns(time)       system_delay((time), (1))          // 设置延时时间  单位ns
//====================================================延时 基础函数====================================================


#endif
