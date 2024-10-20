#ifndef _zf_driver_dma_h_
#define _zf_driver_dma_h_

#include "IfxDma.h"
#include "zf_common_typedef.h"
#include "zf_driver_exti.h"

#define clear_dma_flag(dma_ch)                              (IfxDma_clearChannelInterrupt(&MODULE_DMA, dma_ch))

#define dma_set_destination(dma_ch, destination_addr)       (IfxDma_setChannelDestinationAddress(&MODULE_DMA, (dma_ch), (void *)IFXCPU_GLB_ADDR_DSPR(IfxCpu_getCoreId(), (destination_addr))))

//====================================================DMA 基础函数====================================================
uint8 dma_init      (IfxDma_ChannelId dma_ch, uint8 *source_addr, uint8 *destination_addr, exti_pin_enum eru_pin, exti_trigger_enum trigger, uint16 dma_count);
void  dma_disable   (IfxDma_ChannelId dma_ch);
void  dma_enable    (IfxDma_ChannelId dma_ch);
//====================================================DMA 基础函数====================================================

#endif
