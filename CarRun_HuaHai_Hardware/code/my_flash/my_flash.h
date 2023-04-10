/*
 * my_flash.h
 *
 *  Created on: 2023年3月30日
 *      Author: wzl
 */
#ifndef CODE_MY_FLASH_MY_FLASH_H_
#define CODE_MY_FLASH_MY_FLASH_H_

//包含头文件
#include "cpu0_main.h"

//声明全局变量
extern bool flashSaveEnable;

//====================================================MY_FLASH 基础函数====================================================
void my_flash_init              (void);
void my_flash_write_pid         (float* buf);
void my_flash_Handle            (void);
//====================================================MY_FLASH 基础函数====================================================


#endif /* CODE_MY_FLASH_MY_FLASH_H_ */
