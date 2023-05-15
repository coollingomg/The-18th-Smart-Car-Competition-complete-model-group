/*
 * voltage_sampling.c
 *
 *  Created on: 2023��5��15��
 *      Author: wzl
 */
#include "voltage_sampling/voltage_sampling.h"
#include "Buzzer/buzzer.h"

//����adc�����Ľṹ��
ADC_sampling adc_sampling;


//-------------------------------------------------------------------------------------------------------------------
// �������       adc��ʼ��
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void adc_Init(void)
{
    //��ѹ������ʼ��
    adc_init(ADC0_CH2_A2, ADC_12BIT);

    //��ز�����ʼ��
    adc_sampling.if_adc_handle = false;        //��ʼĬ��ʧ��
    adc_sampling.adc_getNum = 0.0;             //��ʼ����ֵΪ0
}


//-------------------------------------------------------------------------------------------------------------------
// �������       adc�̼߳���������
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void adc_Timer(void)
{
    //û��һ�θú���������������
    adc_sampling.count++;

    //ÿ10000ms����һ��adc��������
    if(adc_sampling.count >= SAMPL_TIME)
    {
        //ʹ��adc���������־λ
        adc_sampling.if_adc_handle = true;
        //���������
        adc_sampling.count = 0;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������       adc�̴߳���
// ����˵��       void
// ���ز���       void
//-------------------------------------------------------------------------------------------------------------------
void adc_Handle(void)
{
    if(adc_sampling.if_adc_handle == true)
    {
        //�����������ȡadc�Ĵ�����ֵ
        uint16 get_value = 0;
        //��ʼ���е�ѹ����������5�ˣ����ز���ƽ��ֵ
        get_value = adc_mean_filter_convert(ADC0_CH2_A2, 5);
        //���õ���ֵת��Ϊ��ѹֵ
        adc_sampling.adc_getNum = get_value / CONVERSION_FAC * MAX_VOLTAGE;
        //�жϵ�ѹ�Ƿ�������õ�ѹ
        if(adc_sampling.adc_getNum < VOLTAGE_VALUE)
        {
            Buzzer_Enable(BuzzerVoltage);
        }
        //�����־λ
        adc_sampling.if_adc_handle = false;
    }
}

