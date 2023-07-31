#ifndef _zf_driver_soft_iic_h_
#define _zf_driver_soft_iic_h_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

typedef struct
{
    uint32              scl_pin;                                                // 用于记录对应的引脚编号
    uint32              sda_pin;                                                // 用于记录对应的引脚编号
    uint8               addr;                                                   // 器件地址 七位地址模式
    uint32              delay;                                                  // 模拟 IIC 软延时时长  0：1370KHz 10：1020KHz 20：757KHz 30: 633KHz  40: 532Khz  50: 448KHz  60: 395KHz  70: 359KHz  80: 324KHz  100: 268KHz  1000：32KHz
    void               *iic_scl;                                                // 记录 SCL 端口地址
    void               *iic_sda;                                                // 记录 SDA 端口地址
}soft_iic_info_struct;


//==================================================SOFT_IIC 基础函数====================================================
void        soft_iic_write_8bit             (soft_iic_info_struct *soft_iic_obj, const uint8 data);
void        soft_iic_write_8bit_array       (soft_iic_info_struct *soft_iic_obj, const uint8 *data, uint32 len);

void        soft_iic_write_16bit            (soft_iic_info_struct *soft_iic_obj, const uint16 data);
void        soft_iic_write_16bit_array      (soft_iic_info_struct *soft_iic_obj, const uint16 *data, uint32 len);

void        soft_iic_write_8bit_register    (soft_iic_info_struct *soft_iic_obj, const uint8 register_name, const uint8 data);
void        soft_iic_write_8bit_registers   (soft_iic_info_struct *soft_iic_obj, const uint8 register_name, const uint8 *data, uint32 len);

void        soft_iic_write_16bit_register   (soft_iic_info_struct *soft_iic_obj, const uint16 register_name, const uint16 data);
void        soft_iic_write_16bit_registers  (soft_iic_info_struct *soft_iic_obj, const uint16 register_name, const uint16 *data, uint32 len);

uint8       soft_iic_read_8bit              (soft_iic_info_struct *soft_iic_obj);
void        soft_iic_read_8bit_array        (soft_iic_info_struct *soft_iic_obj, uint8 *data, uint32 len);

uint16      soft_iic_read_16bit             (soft_iic_info_struct *soft_iic_obj);
void        soft_iic_read_16bit_array       (soft_iic_info_struct *soft_iic_obj, uint16 *data, uint32 len);

uint8       soft_iic_read_8bit_register     (soft_iic_info_struct *soft_iic_obj, const uint8 register_name);
void        soft_iic_read_8bit_registers    (soft_iic_info_struct *soft_iic_obj, const uint8 register_name, uint8 *data, uint32 len);

uint16      soft_iic_read_16bit_register    (soft_iic_info_struct *soft_iic_obj, const uint16 register_name);
void        soft_iic_read_16bit_registers   (soft_iic_info_struct *soft_iic_obj, const uint16 register_name, uint16 *data, uint32 len);

void        soft_iic_sccb_write_register    (soft_iic_info_struct *soft_iic_obj, const uint8 register_name, uint8 data);
uint8       soft_iic_sccb_read_register     (soft_iic_info_struct *soft_iic_obj, const uint8 register_name);

void        soft_iic_init                   (soft_iic_info_struct *soft_iic_obj, uint8 addr, uint32 delay, gpio_pin_enum scl_pin, gpio_pin_enum sda_pin);
//==================================================SOFT_IIC 基础函数====================================================


#endif
