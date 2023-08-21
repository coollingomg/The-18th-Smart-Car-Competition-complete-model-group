/*
 * INA226.h
 *
 *  Created on: 2023年7月28日
 *      Author: wzl
 */

#ifndef CODE_CURRENT_SAMPLING_INA226_H_
#define CODE_CURRENT_SAMPLING_INA226_H_

//包含头文件
#include <stdarg.h>
#include "cpu0_main.h"


// 宏定义地址
#define READ_ADDR                  0x81
#define WRITE_ADDR                 0x80
#define Config_Reg                 0x00
#define Shunt_V_Reg                0x01
#define Bus_V_Reg                  0x02
#define Power_Reg                  0x03
#define Current_Reg                0x04
#define Calib_Reg                  0x05
#define Mask_En_Reg                0x06
#define Alert_Reg                  0x07
#define Man_ID_Reg                 0xFE
#define ID_Reg                     0xFF


//==================================================INA226 基础函数==================================================
void    INA226_Init                  (void);
void    INA226_Timer                 (void);
int16   INA226_Read2Byte             (uint8 reg_addr);
uint8   INA226_Write2Byte            (uint8 reg_addr, uint16 reg_data);
//==================================================INA226 基础函数==================================================

#endif
