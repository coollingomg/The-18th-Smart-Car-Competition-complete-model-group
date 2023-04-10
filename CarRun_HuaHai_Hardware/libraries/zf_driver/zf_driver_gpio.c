#include "zf_driver_gpio.h"

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      获取GPIO基地址
//  参数说明      pin         选择的引脚 (可选择范围由 zf_driver_gpio.h 内gpio_pin_enum枚举值确定)
//  返回参数      void
//  使用示例      get_port(P00_0)
//  备注信息      本函数在文件内部调用 用户不用关注 也不可修改
//-------------------------------------------------------------------------------------------------------------------
Ifx_P* get_port (gpio_pin_enum pin)
{
    volatile Ifx_P *port;

    switch(pin&0xffe0)
    {
        case P00_0: port = &MODULE_P00; break;
        case P02_0: port = &MODULE_P02; break;
        case P10_0: port = &MODULE_P10; break;
        case P11_0: port = &MODULE_P11; break;
        case P13_0: port = &MODULE_P13; break;
        case P14_0: port = &MODULE_P14; break;
        case P15_0: port = &MODULE_P15; break;
        case P20_0: port = &MODULE_P20; break;
        case P21_0: port = &MODULE_P21; break;
        case P22_0: port = &MODULE_P22; break;
        case P23_0: port = &MODULE_P23; break;
        case P32_0: port = &MODULE_P32; break;
        case P33_0: port = &MODULE_P33; break;
        default:break;
    }
#pragma warning 507
    return port;
#pragma warning default
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     gpio 输出设置
// 参数说明     pin         选择的引脚 (可选择范围由 zf_driver_gpio.h 内 gpio_pin_enum 枚举值确定)
// 参数说明     dat         0：低电平 1：高电平
// 返回参数     void
// 使用示例     gpio_set_level(P00_0, 1);// P00_0 输出高电平
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void gpio_set_level (gpio_pin_enum pin, uint8 dat)
{
    if(dat)
    {
        IfxPort_setPinHigh(get_port(pin), pin&0x1f);
    }
    else
    {
        IfxPort_setPinLow(get_port(pin), pin&0x1f);
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     gpio 电平获取
// 参数说明     pin         选择的引脚 (可选择范围由 zf_driver_gpio.h 内 gpio_pin_enum 枚举值确定)
// 返回参数     uint8       引脚当前电平
// 使用示例     uint8 status = gpio_get_level(P00_0);// 获取P00_0引脚电平
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 gpio_get_level (gpio_pin_enum pin)
{
    return IfxPort_getPinState(get_port(pin), pin&0x1f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     gpio 翻转电平
// 参数说明     pin         选择的引脚 (可选择范围由 zf_driver_gpio.h 内 gpio_pin_enum 枚举值确定)
// 返回参数     void
// 使用示例     gpio_toggle_level(P00_0);// P00_0引脚电平翻转
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void gpio_toggle_level (gpio_pin_enum pin)
{
    IfxPort_togglePin(get_port(pin), pin&0x1f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     gpio 方向设置
// 参数说明     pin         选择的引脚 (可选择范围由 zf_driver_gpio.h 内 gpio_pin_enum 枚举值确定)
// 参数说明     dir         引脚的方向   输出：GPO   输入：GPI
// 参数说明     mode        引脚的模式 (可选择范围由 zf_driver_gpio.h 内 gpio_mode_enum 枚举值确定)
// 返回参数     void
// 使用示例     gpio_dir(P00_0, GPO, GPO_PUSH_PULL);// 设置P00_0为推挽输出模式
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void gpio_set_dir (gpio_pin_enum pin, gpio_dir_enum dir, gpio_mode_enum pinmode)
{
    IfxPort_Mode port_mode;
    if(dir == GPI)
    {
        switch(pinmode)
        {
            case GPI_FLOATING_IN:port_mode = IfxPort_Mode_inputNoPullDevice ;       break; // 浮空输入
            case GPI_PULL_DOWN  :port_mode = IfxPort_Mode_inputPullDown     ;       break; // 下拉输入
            default:             port_mode = IfxPort_Mode_inputPullUp       ;       break; // 默认为上拉输入
        }
    }
    else
    {
        switch(pinmode)
        {
            case GPO_OPEN_DTAIN :port_mode = IfxPort_Mode_outputOpenDrainGeneral;   break;// 开漏输出
            default:             port_mode = IfxPort_Mode_outputPushPullGeneral ;   break;// 默认为推挽输出
        }
    }

    IfxPort_setPinMode(get_port(pin), pin&0x1f, port_mode);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     gpio 初始化
// 参数说明     pin         选择的引脚 (可选择范围由 zf_driver_gpio.h 内 gpio_pin_enum 枚举值确定)
// 参数说明     mode        引脚的方向 [GPI/GPIO]
// 参数说明     dat         引脚初始化时设置的电平状态，输出时有效 0：低电平 1：高电平 仅在设置为输出模式时有效
// 参数说明     mode        引脚的模式 (可选择范围由 zf_driver_gpio.h 内 gpio_mode_enum 枚举值确定)
// 返回参数     void
// 使用示例     gpio_init(P00_0, GPO, 1, GPO_PUSH_PULL); // P00_0初始化为GPIO功能、输出模式、输出高电平、推挽输出
// 备注信息     需要特别注意P20_2是不能用于输出的，仅仅只有输入的功能
//-------------------------------------------------------------------------------------------------------------------
void gpio_init (gpio_pin_enum pin, gpio_dir_enum dir, uint8 dat, gpio_mode_enum pinmode)
{
    IfxPort_Mode port_mode;

    if(dir == GPI)
    {
        switch(pinmode)
        {
            case GPI_FLOATING_IN:port_mode = IfxPort_Mode_inputNoPullDevice ;       break; // 浮空输入
            case GPI_PULL_DOWN  :port_mode = IfxPort_Mode_inputPullDown     ;       break; // 下拉输入
            default:             port_mode = IfxPort_Mode_inputPullUp       ;       break; // 默认为上拉输入
        }
    }
    else
    {
        switch(pinmode)
        {
            case GPO_OPEN_DTAIN :port_mode = IfxPort_Mode_outputOpenDrainGeneral;   break;// 开漏输出
            default:             port_mode = IfxPort_Mode_outputPushPullGeneral ;   break;// 默认为推挽输出
        }
    }

    IfxPort_setPinMode(get_port(pin), pin&0x1f, port_mode);

    IfxPort_setPinPadDriver(get_port(pin), pin&0x1f, IfxPort_PadDriver_cmosAutomotiveSpeed1);

    if(dir == GPO)
    {
        if(dat)
        {
            IfxPort_setPinHigh(get_port(pin), pin&0x1f);
        }
        else
        {
            IfxPort_setPinLow(get_port(pin), pin&0x1f);
        }
    }

}
