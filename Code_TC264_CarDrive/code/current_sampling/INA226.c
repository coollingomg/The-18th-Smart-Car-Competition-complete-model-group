/*
 * INA226.c
 *
 *  Created on: 2023年7月28日
 *      Author: wzl
 */

#include "SysSe/Math/Ifx_LowPassPt1F32.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_delay.h"
#include "INA226.h"
#include "motor.h"

// 定义软件IIC参数结构体-->用于对电流计进行数据获取
soft_iic_info_struct Soft_IIC_Struction;
// 定义一阶低通滤波器结构体
Ifx_LowPassPt1F32 LowPass_filter;
// 定义一阶低通滤波器的参数配置结构体
Ifx_LowPassPt1F32_Config LowPass_filter_config;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取ina226数据
// 参数说明     reg_addr            要读取的地址
// 返回参数     reg_data            读取到的数据
//-------------------------------------------------------------------------------------------------------------------
int16 INA226_Read2Byte(uint8_t reg_addr)
{
    // 定义接受数据变量
    int16 reg_data=0;
    bool cur_reg_sign_wdnmd = 0;

    // 定义数组来存储数据
    uint8_t data[2];

    // 读取数据
    soft_iic_read_8bit_registers(&Soft_IIC_Struction, reg_addr ,data ,2);

    // 融合数据
    reg_data= data[0];
    reg_data=(reg_data<<8)&0xFF00;
    reg_data |= data[1];

    //---New Added Start---
    //cur_reg_sign_wdnmd = (reg_data & 0x8000)>>15;
    //if (cur_reg_sign_wdnmd)
    //{
    //   reg_data = ~(reg_data - 1);
    //}
    //---New Added End---

    // 返回数据
    return reg_data;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     向ina226写入数据
// 参数说明     reg_addr            写入数据的地址
// 参数说明     reg_data            写入的数据
// 返回参数     返回状态码
//-------------------------------------------------------------------------------------------------------------------
uint8_t INA226_Write2Byte(uint8_t reg_addr,uint16 reg_data)
{
    // 获取uint16的高低位数据
    uint8_t data_high=(uint8_t)((reg_data&0xFF00)>>8);
    uint8_t data_low=(uint8_t)reg_data&0x00FF;
    uint8_t data[2] = {data_high, data_low};

    // 写入数据
    soft_iic_write_8bit_registers(&Soft_IIC_Struction, reg_addr, data, 2);
    system_delay_ms(2);

    // 写入完成
    return 1;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ina226模式初始化
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void INA226_Init(void)
{
    // 初始化IIC接口
    soft_iic_init(&Soft_IIC_Struction, 0x40, 60, P33_12, P33_11);

    // 采样模式设置
    while(!INA226_Write2Byte(Config_Reg, 0x424D));      // 0100_001_001_001_101；4次平均、0.204ms、0.204ms、连续测量分流电压
    while(!INA226_Write2Byte(Calib_Reg, 0x0800));       // 分辨率为0.0005A，采样电阻为0.005Ω

    // 低通滤波器的初始化
    LowPass_filter_config.cutOffFrequency = 10;                              // 低通滤波器的截止频率为10hz
    LowPass_filter_config.samplingTime = 0.0005;                             // 滤波的数据采样时间为0.0005s
    LowPass_filter_config.gain = 0.85;                                       // 低通滤波器的增益为0.85
    Ifx_LowPassPt1F32_init(&LowPass_filter, &LowPass_filter_config);         // 低通滤波器的初始化
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ina226-->定时读取数据
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void INA226_Timer(void)
{
    // 读取电流数据
    float current_motor = INA226_Read2Byte(Current_Reg) * 0.0005f / 5;  // 测量的采样电压(mv) / 采样电阻值(mΩ)

    if(abs(current_motor) >= 4)
        return;

    motorStr.Current_motor = current_motor;
    // 进行一阶低通滤波
    motorStr.Current_motor_After_filter = Ifx_LowPassPt1F32_do(&LowPass_filter, current_motor);
//    motorStr.Current_motor_After_filter = filterInput(&three_filter_for_current, current_motor);
}

