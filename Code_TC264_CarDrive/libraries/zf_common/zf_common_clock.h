#ifndef _zf_common_clock_h_
#define _zf_common_clock_h_

#include "zf_common_typedef.h"

#define AURIX_MCU_FREQUENCY         (200*1000*1000) // 设置单片机频率为200M 200M(默认频率)

void set_clock              (void);                 // 设置系统频率
void get_clock              (void);                 // 获取系统频率
void disable_Watchdog       (void);                 // 关闭看门狗
void clock_init             (void);                 // 时钟初始化
void cpu_wait_event_ready   (void);                 // 等待所有核心初始化完毕
#endif
