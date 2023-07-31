/*
 * INA226.c
 *
 *  Created on: 2023年7月28日
 *      Author: wzl
 */


#include "INA226.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_delay.h"
#include "motor.h"


// 定义软件IIC参数结构体-->用于对电流计进行数据获取
soft_iic_info_struct Soft_IIC_Struction;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取ina226数据
// 参数说明     reg_addr            要读取的地址
// 返回参数     reg_data            读取到的数据
//-------------------------------------------------------------------------------------------------------------------
int16 INA226_Read2Byte(uint8_t reg_addr)
{
    // 定义接受数据变量
    int16 reg_data=0;

    // 定义数组来存储数据
    uint8_t data[2];

    // 读取数据
    soft_iic_read_8bit_registers(&Soft_IIC_Struction, reg_addr ,data ,2);

    // 融合数据
    reg_data= data[0];
    reg_data=(reg_data<<8)&0xFF00;
    reg_data |= data[1];

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
    while(!INA226_Write2Byte(Config_Reg, 0x4207));      // 0100_001_000_000_111；4次平均、0.14ms、0.14ms、连续测量分流电压和总线电压
    while(!INA226_Write2Byte(Calib_Reg, 0x0A00));       // 分辨率为0.001A，采样电阻为0.002Ω
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ina226-->定时读取数据
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void INA226_Timer(void)
{
    // 读取电流数据
    motorStr.Current_motor = INA226_Read2Byte(Current_Reg) * 0.001f;
}

