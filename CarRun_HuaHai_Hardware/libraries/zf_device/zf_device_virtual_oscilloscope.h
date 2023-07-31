#ifndef _zf_device_virtual_oscilloscope_h_
#define _zf_device_virtual_oscilloscope_h_

#include "zf_common_typedef.h"

extern uint8 virtual_oscilloscope_data[10];

void    virtual_oscilloscope_data_conversion    (const int16 data1, const int16 data2, const int16 data3, const int16 data4);



#endif
