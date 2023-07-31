#include "headfile.h"
#include "rtthread.h"
#pragma section all "cpu0_dsram"
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// ================运行移植提示================
// ====== 当前 RTThread 仅能在 Cpu 0 运行 ======
// ====== 当前 RTThread 仅能在 Cpu 0 运行 ======
// ====== 当前 RTThread 仅能在 Cpu 0 运行 ======
// ================运行移植提示================


//工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译
//工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
//然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
//一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

//对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用enableInterrupts();来开启中断嵌套
//简单点说实际上进入中断后TC系列的硬件自动调用了disableInterrupts();来拒绝响应任何的中断，因此需要我们自己手动调用enableInterrupts();来开启中断的响应。
//函数声明

static rt_uint32_t count = 0;


void thread1_entry (void *parameter);
void thread2_entry (void *parameter);
int critical_section_example(void);

//------------------------------------------------------------
// @brief       线程1入口函数
// @param       parameter   参数
// @return      void
// Sample usage:
//------------------------------------------------------------
void thread1_entry (void *parameter)
{
    while(1)
    {
        // 调度器上锁，上锁后将不再切换到其他线程，仅响应中断
        rt_enter_critical();
        // 以下进入临界区

        count += 10;                                        // 计数值+10

        // 调度器解锁
        rt_exit_critical();

        rt_kprintf("thread = %d , count = %d\n", 10, count);
        rt_thread_mdelay(1000);
    }
}

//------------------------------------------------------------
// @brief       线程2入口函数
// @param       parameter   参数
// @return      void
// Sample usage:
//------------------------------------------------------------
void thread2_entry (void *parameter)
{
    while(1)
    {
        // 调度器上锁，上锁后将不再切换到其他线程，仅响应中断
        rt_enter_critical();
        // 以下进入临界区

        count += 20;                                        // 计数值+20

        // 调度器解锁
        rt_exit_critical();

        rt_kprintf("thread = %d , count = %d\n", 20, count);
        rt_thread_mdelay(2000);
    }
}

//------------------------------------------------------------
// @brief       线程创建以及启动
// @param       void        空
// @return      void
// Sample usage:
//------------------------------------------------------------
int critical_section_example(void)
{
    // 线程控制块指针
    rt_thread_t tid;
    // 创建动态线程
    tid = rt_thread_create("thread_10",                     // 线程名称
        thread1_entry,                                      // 线程入口函数
        RT_NULL,                                            // 线程参数
        256,                                                // 栈空间
        5,                                                  // 线程优先级为5，数值越小，优先级越高，0为最高优先级。
                                                            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
        10);                                                // 时间片

    if(tid != RT_NULL)                                      // 线程创建成功
    {
        // 运行该线程
        rt_thread_startup(tid);
    }

    tid = rt_thread_create("thread_20",                     // 线程名称
        thread2_entry,                                      // 线程入口函数
        RT_NULL,                                            // 线程参数
        256,                                                // 栈空间
        3,                                                  // 线程优先级为3，数值越小，优先级越高，0为最高优先级。
                                                            // 可以通过修改rt_config.h中的RT_THREAD_PRIORITY_MAX宏定义(默认值为8)来修改最大支持的优先级
        10);                                                // 时间片

    if(tid != RT_NULL)                                      // 线程创建成功
    {
        rt_thread_startup(tid);                             // 运行该线程
    }

    return 0;
}

// 使用INIT_APP_EXPORT宏自动初始化，也可以通过在其他线程内调用critical_section_example函数进行初始化
INIT_APP_EXPORT(critical_section_example);                  // 应用初始化

int main(void)
{


    //等待所有核心初始化完毕
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);



    //初始化LED引脚
    gpio_init(P20_8, GPO, 1, PUSHPULL);
    gpio_init(P20_9, GPO, 1, PUSHPULL);
    gpio_init(P21_4, GPO, 1, PUSHPULL);
    gpio_init(P21_5, GPO, 1, PUSHPULL);


    while(1)
    {
        //翻转LED引脚
        gpio_toggle(P20_8);
        gpio_toggle(P20_9);
        gpio_toggle(P21_4);
        gpio_toggle(P21_5);
        rt_thread_mdelay(500);
    }
}

#pragma section all restore


