#include "IfxStm.h"
#include "IFXSTM_CFG.h"
#include "zf_driver_timer.h"

static uint32 systick_count[2];

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      systick定时器启动
//  返回参数      void
//  使用示例      system_start(); // 启动定时器，记录下当前的时间
//-------------------------------------------------------------------------------------------------------------------
void system_start (void)
{
    systick_count[(IfxCpu_getCoreId())] = IfxStm_getLower(IfxStm_getAddress((IfxStm_Index)(IfxCpu_getCoreId())));
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     获得当前System tick timer的值
//  返回参数     uint32           返回从开始到现在的时间(单位10ns)
//  使用示例     uint32 tim = system_getval();
//  备注信息     在核心0调用此函数则使用STM0模块  核心1则使用STM1模块
//-------------------------------------------------------------------------------------------------------------------
uint32 system_getval (void)
{
    uint32 time;
    uint32 stm_clk;

    stm_clk = IfxStm_getFrequency(IfxStm_getAddress((IfxStm_Index)(IfxCpu_getCoreId())));

    time = IfxStm_getLower(IfxStm_getAddress((IfxStm_Index)(IfxCpu_getCoreId())));
    time = time - systick_count[(IfxCpu_getCoreId())];
    time = (uint32)((uint64)time * 100000000 / stm_clk);

    return time;
}




