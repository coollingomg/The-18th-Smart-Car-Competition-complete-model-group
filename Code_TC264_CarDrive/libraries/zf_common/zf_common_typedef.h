#ifndef _zf_common_typedef_h_
#define _zf_common_typedef_h_

#include "math.h"
#include "stdio.h"
#include "stdint.h"
#include "stdarg.h"
#include "string.h"
#include "stdlib.h"
#include "ifx_types.h"
#include "PLATFORM_TYPES.H"
//=================================================== 类型定义 ===================================================

//#define COMPATIBLE_WITH_OLDER_VERSIONS                                          // 兼容旧版开源库接口
#define USE_ZF_TYPEDEF      (1)                                                 // 是否启用类型定义申明
#if USE_ZF_TYPEDEF
// 数据类型声明
// 尽量使用 stdint.h 定义的类型名称 避免冲突 这里可以裁剪
typedef signed char         int8;                                               // 有符号  8 bits
typedef signed short int    int16;                                              // 有符号 16 bits
typedef signed int          int32;                                              // 有符号 32 bits
typedef signed long long    int64;                                              // 有符号 64 bits

typedef volatile uint8      vuint8;                                             // 易变性修饰 无符号  8 bits
typedef volatile uint16     vuint16;                                            // 易变性修饰 无符号 16 bits
typedef volatile uint32     vuint32;                                            // 易变性修饰 无符号 32 bits
typedef volatile uint64     vuint64;                                            // 易变性修饰 无符号 64 bits

typedef volatile int8       vint8;                                              // 易变性修饰 有符号  8 bits
typedef volatile int16      vint16;                                             // 易变性修饰 有符号 16 bits
typedef volatile int32      vint32;                                             // 易变性修饰 有符号 32 bits
typedef volatile int64      vint64;                                             // 易变性修饰 有符号 64 bits
#endif

#define ZF_ENABLE       (1)
#define ZF_DISABLE      (0)

#define ZF_TRUE         (1)
#define ZF_FALSE        (0)
//=================================================== 类型定义 ===================================================

#endif
