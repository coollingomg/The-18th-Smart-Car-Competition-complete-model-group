#include "IfxScuEru.h"
#include "Ifxstm.h"
#include "Cpu0_Main.h"
#include "Cpu/Std/IfxCpu.h"
#include "zf_common_interrupt.h"
#include "zf_common_clock.h"

App_Cpu0 g_AppCpu0;                               // 频率信息变量

IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0; // 事件同步变量

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          设置系统频率
// 返回类型          void
// 使用示例          set_clock();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void set_clock (void)
{
    IfxScuCcu_setCpuFrequency(IfxCpu_ResourceCpu_0, (float32)AURIX_MCU_FREQUENCY);
    IfxScuCcu_setCpuFrequency(IfxCpu_ResourceCpu_1, (float32)AURIX_MCU_FREQUENCY);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          获取系统频率
// 返回类型          void
// 使用示例          get_clock();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void get_clock (void)
{
    // 获取时钟频率，便于查看当前系统运行频率
    g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
    g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
    g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
    g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          关闭看门狗
// 返回类型          void
// 使用示例          disable_Watchdog();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void disable_Watchdog (void)
{
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          时钟初始化
// 返回类型          void
// 使用示例          clock_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void clock_init (void)
{
    interrupt_global_disable();     // 关闭总中断
    disable_Watchdog();             // 关闭看门狗
    get_clock();                    // 获取系统频率
    interrupt_global_enable(0);     // 打开全局中断
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介          等待所有核心初始化完毕
// 返回类型          void
// 使用示例          cpu_wait_event_ready();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void cpu_wait_event_ready (void)
{
    while(IfxCpu_waitEvent(&g_cpuSyncEvent, 5))
    {
        IfxCpu_emitEvent(&g_cpuSyncEvent);
    }
}
