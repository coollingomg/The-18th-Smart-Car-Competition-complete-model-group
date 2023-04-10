#ifndef _zf_common_interrupt_h_
#define _zf_common_interrupt_h_

#include "zf_common_typedef.h"

void   interrupt_global_enable (uint32 primask);    // 全局中断使能
uint32 interrupt_global_disable (void);             // 全局中断失能
void   assert_interrupt_config  (void);             // 断言中断配置

#endif

