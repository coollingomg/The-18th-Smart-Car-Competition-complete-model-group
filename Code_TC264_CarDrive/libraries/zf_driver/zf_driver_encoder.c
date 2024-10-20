#include "IfxGpt12_IncrEnc.h"
#include "zf_common_debug.h"
#include "zf_driver_encoder.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器地址设置 内部调用
// 参数说明     gptn            选择所使用的GPT12定时器
// 参数说明     count_pin       设置计数引脚
// 参数说明     dir_pin         设置计数方向引脚
// 返回参数     void
// 使用示例     encoder_mapping_set(gptn, ch1_pin, ch2_pin);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static void encoder_mapping_set (encoder_index_enum encoder_n, encoder_channel1_enum ch1_pin, encoder_channel2_enum ch2_pin)
{
    IfxGpt12_TxIn_In  *ch1;
    IfxGpt12_TxEud_In *ch2;

    switch(encoder_n)
    {
        case TIM2_ENCODER:
        {
            if      (TIM2_ENCODER_CH1_P00_7  == ch1_pin)    ch1   = &IfxGpt120_T2INA_P00_7_IN;
            else if (TIM2_ENCODER_CH1_P33_7  == ch1_pin)    ch1   = &IfxGpt120_T2INB_P33_7_IN;
            else    zf_assert(FALSE);

            if      (TIM2_ENCODER_CH2_P00_8  == ch2_pin)    ch2   = &IfxGpt120_T2EUDA_P00_8_IN;
            else if (TIM2_ENCODER_CH2_P33_6  == ch2_pin)    ch2   = &IfxGpt120_T2EUDB_P33_6_IN;
            else    zf_assert(FALSE);
        }break;

        case TIM3_ENCODER:
        {
            if      (TIM3_ENCODER_CH1_P02_6  == ch1_pin)    ch1   = &IfxGpt120_T3INA_P02_6_IN;
            else    zf_assert(FALSE);

            if      (TIM3_ENCODER_CH2_P02_7  == ch2_pin)    ch2   = &IfxGpt120_T3EUDA_P02_7_IN;
            else    zf_assert(FALSE);
        }break;

        case TIM4_ENCODER:
        {
            if      (TIM4_ENCODER_CH1_P02_8  == ch1_pin)    ch1   = &IfxGpt120_T4INA_P02_8_IN;
            else    zf_assert(FALSE);

            if      (TIM4_ENCODER_CH2_P00_9  == ch2_pin)    ch2   = &IfxGpt120_T4EUDA_P00_9_IN;
            else if (TIM4_ENCODER_CH2_P33_5  == ch2_pin)    ch2   = &IfxGpt120_T4EUDB_P33_5_IN;
            else    zf_assert(FALSE);
        }break;

        case TIM5_ENCODER:
        {
            if      (TIM5_ENCODER_CH1_P21_7  == ch1_pin)    ch1   = &IfxGpt120_T5INA_P21_7_IN;
            else if (TIM5_ENCODER_CH1_P10_3  == ch1_pin)    ch1   = &IfxGpt120_T5INB_P10_3_IN;
            else    zf_assert(FALSE);

            if      (TIM5_ENCODER_CH2_P21_6  == ch2_pin)    ch2   = &IfxGpt120_T5EUDA_P21_6_IN;
            else if (TIM5_ENCODER_CH2_P10_1  == ch2_pin)    ch2   = &IfxGpt120_T5EUDB_P10_1_IN;
            else    zf_assert(FALSE);
        }break;

        case TIM6_ENCODER:
        {
            if      (TIM6_ENCODER_CH1_P20_3  == ch1_pin)    ch1   = &IfxGpt120_T6INA_P20_3_IN;
            else if (TIM6_ENCODER_CH1_P10_2  == ch1_pin)    ch1   = &IfxGpt120_T6INB_P10_2_IN;
            else    zf_assert(FALSE);

            if      (TIM6_ENCODER_CH2_P20_0  == ch2_pin)    ch2   = &IfxGpt120_T6EUDA_P20_0_IN;
            else    zf_assert(FALSE);
        }break;
    }
#pragma warning 507

    IfxGpt12_initTxInPinWithPadLevel(ch1,  IfxPort_InputMode_pullUp, IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGpt12_initTxEudInPinWithPadLevel(ch2, IfxPort_InputMode_pullUp, IfxPort_PadDriver_cmosAutomotiveSpeed1);

#pragma warning default
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      编码器数据采集
//  参数说明      gptn            选择所使用的定时器
//  返回参数      void
//  使用示例      int16 speed; speed = encoder_get_count(TIM2_ENCODER); // 使用T2定时器
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
int16 encoder_get_count (encoder_index_enum encoder_n)
{
    int16 encoder_data = 0;
    switch(encoder_n)
    {
        case TIM2_ENCODER: encoder_data = (int16)IfxGpt12_T2_getTimerValue(&MODULE_GPT120); break;
        case TIM3_ENCODER: encoder_data = (int16)IfxGpt12_T3_getTimerValue(&MODULE_GPT120); break;
        case TIM4_ENCODER: encoder_data = (int16)IfxGpt12_T4_getTimerValue(&MODULE_GPT120); break;
        case TIM5_ENCODER: encoder_data = (int16)IfxGpt12_T5_getTimerValue(&MODULE_GPT120); break;
        case TIM6_ENCODER: encoder_data = (int16)IfxGpt12_T6_getTimerValue(&MODULE_GPT120); break;
        default: encoder_data = 0;
    }
    return encoder_data;
}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介      编码器计数清除
//  参数说明      gptn            选择所使用的定时器
//  返回参数      void
//  使用示例      encoder_clear_count(TIM2_ENCODER);// 使用T2定时器
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void encoder_clear_count (encoder_index_enum encoder_n)
{
    switch(encoder_n)
    {
        case TIM2_ENCODER: IfxGpt12_T2_setTimerValue(&MODULE_GPT120, 0); break;
        case TIM3_ENCODER: IfxGpt12_T3_setTimerValue(&MODULE_GPT120, 0); break;
        case TIM4_ENCODER: IfxGpt12_T4_setTimerValue(&MODULE_GPT120, 0); break;
        case TIM5_ENCODER: IfxGpt12_T5_setTimerValue(&MODULE_GPT120, 0); break;
        case TIM6_ENCODER: IfxGpt12_T6_setTimerValue(&MODULE_GPT120, 0); break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      正交编码器采集初始化
//  参数说明      encoder_n       选择所使用的GPT12定时器
//  参数说明      ch1_pin         ENCODER 通道 1
//  参数说明      ch2_pin         ENCODER 通道 2
//  返回参数      void
//  使用示例      encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P00_7, TIM2_ENCODER_CH2_P00_8);// 使用T2定时器   P00_7引脚为A通道    P00_8引脚为B通道
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void encoder_quad_init (encoder_index_enum encoder_n, encoder_channel1_enum count_pin, encoder_channel2_enum dir_pin)
{
    IfxGpt12_enableModule(&MODULE_GPT120);
    IfxGpt12_setGpt1BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt1BlockPrescaler_4);
    IfxGpt12_setGpt2BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt2BlockPrescaler_4);
    encoder_mapping_set(encoder_n, count_pin, dir_pin);

    switch(encoder_n)
    {
        case TIM2_ENCODER:
        {
            IfxGpt12_T2_setCounterInputMode(&MODULE_GPT120, IfxGpt12_CounterInputMode_risingEdgeTxIN);
            IfxGpt12_T2_setDirectionSource (&MODULE_GPT120, IfxGpt12_TimerDirectionSource_external);
            IfxGpt12_T2_setMode            (&MODULE_GPT120, IfxGpt12_Mode_counter);
            IfxGpt12_T2_run                (&MODULE_GPT120, IfxGpt12_TimerRun_start);
        }break;

        case TIM3_ENCODER:
        {
            IfxGpt12_T3_setCounterInputMode(&MODULE_GPT120, IfxGpt12_CounterInputMode_risingEdgeTxIN);
            IfxGpt12_T3_setDirectionSource (&MODULE_GPT120, IfxGpt12_TimerDirectionSource_external);
            IfxGpt12_T3_setMode            (&MODULE_GPT120, IfxGpt12_Mode_counter);
            IfxGpt12_T3_run                (&MODULE_GPT120, IfxGpt12_TimerRun_start);
        }break;

        case TIM4_ENCODER:
        {
            IfxGpt12_T4_setCounterInputMode(&MODULE_GPT120, IfxGpt12_CounterInputMode_risingEdgeTxIN);
            IfxGpt12_T4_setDirectionSource (&MODULE_GPT120, IfxGpt12_TimerDirectionSource_external);
            IfxGpt12_T4_setMode            (&MODULE_GPT120, IfxGpt12_Mode_counter);
            IfxGpt12_T4_run                (&MODULE_GPT120, IfxGpt12_TimerRun_start);
        }break;

        case TIM5_ENCODER:
        {
            IfxGpt12_T5_setCounterInputMode(&MODULE_GPT120, IfxGpt12_CounterInputMode_risingEdgeTxIN);
            IfxGpt12_T5_setDirectionSource (&MODULE_GPT120, IfxGpt12_TimerDirectionSource_external);
            IfxGpt12_T5_setMode            (&MODULE_GPT120, IfxGpt12_Mode_counter);
            IfxGpt12_T5_run                (&MODULE_GPT120, IfxGpt12_TimerRun_start);
        }break;

        case TIM6_ENCODER:
        {
            IfxGpt12_T6_setCounterInputMode(&MODULE_GPT120, IfxGpt12_CounterInputMode_risingEdgeTxIN);
            IfxGpt12_T6_setDirectionSource (&MODULE_GPT120, IfxGpt12_TimerDirectionSource_external);
            IfxGpt12_T6_setMode            (&MODULE_GPT120, IfxGpt12_Mode_counter);
            IfxGpt12_T6_run                (&MODULE_GPT120, IfxGpt12_TimerRun_start);
        }break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      方向编码器采集初始化
//  参数说明      encoder_n       选择所使用的GPT12定时器
//  参数说明      ch1_pin         设置计数引脚
//  参数说明      ch2_pin         设置方向引脚
//  返回参数      void
//  使用示例      encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P00_7, TIM2_ENCODER_CH2_P00_8);// 使用T2定时器   P00_7引脚进行计数    计数方向使用P00_8引脚
//  备注信息      英飞凌系列单片机无需区分正交和方向编码器，此处仅保留接口方便用户使用
//-------------------------------------------------------------------------------------------------------------------
void encoder_dir_init (encoder_index_enum encoder_n, encoder_channel1_enum ch1_pin, encoder_channel2_enum ch2_pin)
{
    encoder_quad_init(encoder_n, ch1_pin, ch2_pin);
}

