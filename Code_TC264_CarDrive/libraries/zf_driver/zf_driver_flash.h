#ifndef _zf_driver_flash_h_
#define _zf_driver_flash_h_

#include "ifxFlash_cfg.h"
#include "zf_common_typedef.h"

#define EEPROM_BASE_ADDR             (IFXFLASH_DFLASH_START)
#define EEPROM_SIZE                  (IFXFLASH_DFLASH_SIZE)                     // 一共有96KB
#define EEPROM_PAGE_SIZE             (EEPROM_SIZE / IFXFLASH_DFLASH_NUM_LOG_SECTORS)
#define EEPROM_PAGE_NUM              (IFXFLASH_DFLASH_NUM_LOG_SECTORS)          // 96KB分为了12页

#define FLASH_DATA_SIZE              (IFXFLASH_DFLASH_PAGE_LENGTH)
#define EEPROM_PAGE_LENGTH           (EEPROM_PAGE_SIZE/FLASH_DATA_SIZE)         // 每页可以存1024个uint32类型的数据


typedef union                                                                   // 固定的数据缓冲单元格式
{
    float   float_type;                                                         // float  类型
    uint32  uint32_type;                                                        // uint32 类型
    int32   int32_type;                                                         // int32  类型
    uint16  uint16_type;                                                        // uint16 类型
    int16   int16_type;                                                         // int16  类型
    uint8   uint8_type;                                                         // uint8  类型
    int8    int8_type;                                                          // int8   类型
}flash_data_union;                                                              // 所有类型数据共用同一个 32bit 地址

extern flash_data_union flash_union_buffer[EEPROM_PAGE_LENGTH];

//====================================================FLASH 基础函数====================================================
uint8   flash_check                     (uint32 sector_num, uint32 page_num);                                   // 校验FLASH页是否有数据
void    flash_erase_page                (uint32 sector_num, uint32 page_num);                                   // 擦除页
void    flash_read_page                 (uint32 sector_num, uint32 page_num, uint32 *buf, uint16 len);          // 读取一页
void    flash_write_page                (uint32 sector_num, uint32 page_num, const uint32 *buf, uint16 len);    // 编程一页
void    flash_read_page_to_buffer       (uint32 sector_num, uint32 page_num);                                   // 从指定 FLASH 的指定页码读取数据到缓冲区
uint8   flash_write_page_from_buffer    (uint32 sector_num, uint32 page_num);                                   // 向指定 FLASH 的扇区的指定页码写入缓冲区的数据
void    flash_buffer_clear              (void);                                                                 // 清空数据缓冲区
//====================================================FALSH 基础函数====================================================


#endif
