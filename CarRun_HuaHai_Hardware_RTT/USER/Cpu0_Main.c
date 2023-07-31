#include "headfile.h"
#include "rtthread.h"
#pragma section all "cpu0_dsram"
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// ================������ֲ��ʾ================
// ====== ��ǰ RTThread ������ Cpu 0 ���� ======
// ====== ��ǰ RTThread ������ Cpu 0 ���� ======
// ====== ��ǰ RTThread ������ Cpu 0 ���� ======
// ================������ֲ��ʾ================


//���̵��뵽���֮��Ӧ��ѡ�й���Ȼ����refreshˢ��һ��֮���ٱ���
//����Ĭ������Ϊ�ر��Ż��������Լ��һ�����ѡ��properties->C/C++ Build->Setting
//Ȼ�����Ҳ�Ĵ������ҵ�C/C++ Compiler->Optimization->Optimization level�������Ż��ȼ�
//һ��Ĭ���½����Ĺ��̶���Ĭ�Ͽ�2���Ż�����˴��Ҳ��������Ϊ2���Ż�

//����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ��enableInterrupts();�������ж�Ƕ��
//�򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ�������disableInterrupts();���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ�����enableInterrupts();�������жϵ���Ӧ��
//��������

static rt_uint32_t count = 0;


void thread1_entry (void *parameter);
void thread2_entry (void *parameter);
int critical_section_example(void);

//------------------------------------------------------------
// @brief       �߳�1��ں���
// @param       parameter   ����
// @return      void
// Sample usage:
//------------------------------------------------------------
void thread1_entry (void *parameter)
{
    while(1)
    {
        // �����������������󽫲����л��������̣߳�����Ӧ�ж�
        rt_enter_critical();
        // ���½����ٽ���

        count += 10;                                        // ����ֵ+10

        // ����������
        rt_exit_critical();

        rt_kprintf("thread = %d , count = %d\n", 10, count);
        rt_thread_mdelay(1000);
    }
}

//------------------------------------------------------------
// @brief       �߳�2��ں���
// @param       parameter   ����
// @return      void
// Sample usage:
//------------------------------------------------------------
void thread2_entry (void *parameter)
{
    while(1)
    {
        // �����������������󽫲����л��������̣߳�����Ӧ�ж�
        rt_enter_critical();
        // ���½����ٽ���

        count += 20;                                        // ����ֵ+20

        // ����������
        rt_exit_critical();

        rt_kprintf("thread = %d , count = %d\n", 20, count);
        rt_thread_mdelay(2000);
    }
}

//------------------------------------------------------------
// @brief       �̴߳����Լ�����
// @param       void        ��
// @return      void
// Sample usage:
//------------------------------------------------------------
int critical_section_example(void)
{
    // �߳̿��ƿ�ָ��
    rt_thread_t tid;
    // ������̬�߳�
    tid = rt_thread_create("thread_10",                     // �߳�����
        thread1_entry,                                      // �߳���ں���
        RT_NULL,                                            // �̲߳���
        256,                                                // ջ�ռ�
        5,                                                  // �߳����ȼ�Ϊ5����ֵԽС�����ȼ�Խ�ߣ�0Ϊ������ȼ���
                                                            // ����ͨ���޸�rt_config.h�е�RT_THREAD_PRIORITY_MAX�궨��(Ĭ��ֵΪ8)���޸����֧�ֵ����ȼ�
        10);                                                // ʱ��Ƭ

    if(tid != RT_NULL)                                      // �̴߳����ɹ�
    {
        // ���и��߳�
        rt_thread_startup(tid);
    }

    tid = rt_thread_create("thread_20",                     // �߳�����
        thread2_entry,                                      // �߳���ں���
        RT_NULL,                                            // �̲߳���
        256,                                                // ջ�ռ�
        3,                                                  // �߳����ȼ�Ϊ3����ֵԽС�����ȼ�Խ�ߣ�0Ϊ������ȼ���
                                                            // ����ͨ���޸�rt_config.h�е�RT_THREAD_PRIORITY_MAX�궨��(Ĭ��ֵΪ8)���޸����֧�ֵ����ȼ�
        10);                                                // ʱ��Ƭ

    if(tid != RT_NULL)                                      // �̴߳����ɹ�
    {
        rt_thread_startup(tid);                             // ���и��߳�
    }

    return 0;
}

// ʹ��INIT_APP_EXPORT���Զ���ʼ����Ҳ����ͨ���������߳��ڵ���critical_section_example�������г�ʼ��
INIT_APP_EXPORT(critical_section_example);                  // Ӧ�ó�ʼ��

int main(void)
{


    //�ȴ����к��ĳ�ʼ�����
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);



    //��ʼ��LED����
    gpio_init(P20_8, GPO, 1, PUSHPULL);
    gpio_init(P20_9, GPO, 1, PUSHPULL);
    gpio_init(P21_4, GPO, 1, PUSHPULL);
    gpio_init(P21_5, GPO, 1, PUSHPULL);


    while(1)
    {
        //��תLED����
        gpio_toggle(P20_8);
        gpio_toggle(P20_9);
        gpio_toggle(P21_4);
        gpio_toggle(P21_5);
        rt_thread_mdelay(500);
    }
}

#pragma section all restore


