/*
 * uart.c
 *
 *  Created on: 2023年3月18日
 *      Author: wzl
 */


//包含头文件
#include "uart.h"
#include "pid/pid.h"
#include "isr_config.h"
#include "timer/timer.h"
#include "servo/servo.h"
#include "motor/motor.h"
#include "Buzzer/buzzer.h"
#include "my_flash/my_flash.h"
#include "icm20602_data_handle.h"
#include "car_control/car_control.h"


//定义蓝牙数据结构体
BlueTooth_data_recevie_Struct Bluetooth_data;
//定义无线串口数据结构体
Wireless_data_recevie_Struct Wireless_data;
//定义eb通信数据结构体
UsbStruct usbStr;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用于与上位机通信的串口初始化
// 参数说明     uartn           串口模块号(UART_0,UART_1,UART_2,UART_3)
// 参数说明     baud            串口波特率
// 参数说明     tx_pin          串口发送引脚
// 参数说明     rx_pin          串口接收引脚
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void USB_uart_init(uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin)
{
    // 串口初始化
    uart_init(uartn, baud, tx_pin, rx_pin);

    // 开启串口中断
    uart_tx_interrupt(uartn, 1);
    uart_rx_interrupt(uartn, 1);

    // UsbStruct数据初始化
    usbStr.counter = 0;                         //连接时间计数器初始化为0
    usbStr.receiveFinished = false;
    usbStr.receiveStart = false;
    usbStr.receiveIndex = 0;
    usbStr.connected = false;                   //初始化默认未与上位机连接
    usbStr.inspectorEnable = false;
    usbStr.counterSend_upper = 0;               //初始化发送速度的计时器
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用于无线串口调试的串口初始化
// 参数说明     uartn           串口模块号(UART_0,UART_1,UART_2,UART_3)
// 参数说明     baud            串口波特率
// 参数说明     tx_pin          串口发送引脚
// 参数说明     rx_pin          串口接收引脚
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void Wireless_uart_init(uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin)
{
    // 串口初始化
    uart_init(uartn, baud, tx_pin, rx_pin);

    // Bluetooth_data数据初始化
    Wireless_data.Flag_Wireless = false;       //无线串口定时发送标志位
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用于无线串口调试的串口初始化
// 参数说明     uartn           串口模块号(UART_0,UART_1,UART_2,UART_3)
// 参数说明     baud            串口波特率
// 参数说明     tx_pin          串口发送引脚
// 参数说明     rx_pin          串口接收引脚
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void Bluetooth_uart_init(uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin)
{
    // 串口初始化
    uart_init(uartn, baud, tx_pin, rx_pin);

    // 开启串口中断
    uart_tx_interrupt(uartn, 1);
    uart_rx_interrupt(uartn, 1);

    // Bluetooth_data数据初始化
    Bluetooth_data.receiveFinished = false;
    Bluetooth_data.receiveStart = false;
    Bluetooth_data.receiveIndex = 0;
    Bluetooth_data.data_verity = 0;
    Bluetooth_data.data_choice = false;         //默认闭环模式
    Bluetooth_data.copyready = false;           //初始时数据未接收到
    Bluetooth_data.Flag_Bluetooth = false;      //无线串口定时发送标志位
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     通过蓝牙向手机发送数据，支持 int、float 数据
// 参数说明     string  将要发送的所有数据写入
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void Bluetooth_Send(char *string, ...)
{
    uart_write_byte_wait(UART_0,0XA5);          //发送数据包头
    char *ptr_string = string;                  //避免直接使用string，导致string指向的地址发生变化
    int int_data;
    float float_data;
    void* void_ptr = &float_data;
    uint8_t* float_buff = (uint8_t*)void_ptr;
    va_list ap;                                 //定义一个va_list类型变量ap
    va_start(ap, string);                       //初始化va，使va指向string的下一个地址
    uint8_t sum = 0x00;                         //定义和校验数据位
    uint8_t sum_buff = 0;                       //处理32位转换为8位
    while(*ptr_string != '\0')
    {
        if(*ptr_string == '%')                  //格式控制符
        {
            switch(*++ptr_string)
            {
                case 'd':
                    int_data = va_arg(ap, int);  //检索变量
                    for (int i = 0; i < 4; i++)
                    {
                        sum_buff = (uint32_t)int_data & (0xFF << 8*i);
                        sum += sum_buff;
                        uart_write_byte_wait(UART_0,sum_buff);
                    }
                    break;
                case 'f':
                    float_data = va_arg(ap, float);
                    for (int i = 0; i < 4; i++)
                    {
                        sum_buff = float_buff[i];
                        sum += sum_buff;
                        uart_write_byte_wait(UART_0,sum_buff);
                    }
                    break;
                default:
                    break;
            }
        }
        ptr_string++;  //指向下一个地址
    }
    uart_write_byte_wait(UART_0,sum);       //发送和校验
    uart_write_byte_wait(UART_0,0x5A);      //发送数据包尾
    va_end(ap);                             //关闭ap
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     通过无线串口发送数据，用匿名上位机对指定数据进行监视分析
// 参数说明     data 要发送的数据
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void Wireless_Uart_Send(int32_t data1,int32_t data2,int32_t data3,int32_t data4)
{
    int cnt=0;
    unsigned char i;
    unsigned char sum = 0;
    unsigned char add = 0;
    uint8_t buff[28] = {0};

    buff[cnt++]  = 0xAA;
    buff[cnt++]  = 0xFF;
    buff[cnt++]  = 0xF1;
    buff[cnt++]  = 0x16;

    Byte4_Union byte_4_union;

    byte_4_union.U32 = (uint32_t)data1;
    for(int i=0;i<4;i++)
    {
        buff[cnt++]=byte_4_union.U8_Buff[i];
    }

    byte_4_union.U32 = (uint32_t)data2;
    for(int i=0;i<4;i++)
    {
        buff[cnt++]=byte_4_union.U8_Buff[i];
    }

    byte_4_union.U32 = (uint32_t)data3;
    for(int i=0;i<4;i++)
    {
        buff[cnt++]=byte_4_union.U8_Buff[i];
    }

    byte_4_union.U32 = (uint32_t)data4;
    for(int i=0;i<4;i++)
    {
        buff[cnt++]=byte_4_union.U8_Buff[i];
    }

    for( i=0;i<6;i++)
    {
        buff[cnt++]  = 0x00;
    }

    for( i=0;i<cnt;i++)
    {
        sum += buff[i];
        add += sum;
    }

    buff[cnt++]=sum;
    buff[cnt++]=add;

    uart_write_buffer(UART_0,buff,cnt);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       与edgeboard通信的函数接口，数据包接受函数
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void uart2_rx_interrupt_handler(void)
{
    //定义一个变量来接收数据，并处理
    uint8 temp = 0;
    //接收数据查询式,有数据会返回TRUE,没有数据会返回FALSE.
    if(uart_query_byte(eb_using_uart, &temp))
    {
        //检测到帧头
        if(temp == USB_FRAME_HEAD && !usbStr.receiveStart)
        {
            //开始接收一帧数据
            usbStr.receiveStart = true;
            //写入数据，并指向下一列
            usbStr.receiveBuff[0] = temp;
            usbStr.receiveIndex = 1;
            //默认帧长度
            usbStr.receiveBuff[2] = USB_FRAME_LENMIN;
        }
        else if(usbStr.receiveIndex == 2)
        {
            //接收帧长度数据
            usbStr.receiveBuff[usbStr.receiveIndex] = temp;
            usbStr.receiveIndex++;
            //帧长错误检测
            if(temp > USB_FRAME_LENMAX || temp < USB_FRAME_LENMIN)
            {
                //如果错误，停止接收数据，默认帧长为最小
                usbStr.receiveBuff[2] = USB_FRAME_LENMIN;
                usbStr.receiveIndex = 0;
                usbStr.receiveStart = false;
            }
        }
        else if(usbStr.receiveStart && usbStr.receiveIndex < USB_FRAME_LENMAX)
        {
            //接收具体的数据信息
            usbStr.receiveBuff[usbStr.receiveIndex] = temp;
            usbStr.receiveIndex++;
        }

        //一帧数据接收完毕
        if((usbStr.receiveIndex >= USB_FRAME_LENMAX || usbStr.receiveIndex >= usbStr.receiveBuff[2]) && usbStr.receiveIndex > USB_FRAME_LENMIN)
        {
            //清零和校验位
            usbStr.data_verity = 0;
            //定义帧长变量，默认为最小
            uint8 length = USB_FRAME_LENMIN;
            //获取帧长
            length = usbStr.receiveBuff[2];
            //进行和校验相加
            for(int i=0; i<length-1; i++)
                usbStr.data_verity += usbStr.receiveBuff[i];
            //进行和校验
            if(usbStr.data_verity == usbStr.receiveBuff[length-1])
            {
                //校验成功，数据正确，拷贝数据到receiveBuffFinished，并改变接收数据状态
                memcpy(usbStr.receiveBuffFinished,usbStr.receiveBuff,USB_FRAME_LENMAX);
                usbStr.receiveFinished = true;

                //智能车控制指令特殊处理（保障实时性），USB_ADDR_CONTROL->速度和方向指令
                if(usbStr.receiveBuffFinished[1]  == USB_ADDR_CONTROL)
                {
                    //定义联合体来进行数据处理
                    Byte2_Union byte_2_union;
                    Byte4_Union byte_4_union;
                    //将速度数据赋值给联合体
                    for(int i=0; i<4; i++)
                        byte_4_union.U8_Buff[i] = usbStr.receiveBuffFinished[3+i];
                    //将方向数据赋值给联合体
                    byte_2_union.U8_Buff[0] = usbStr.receiveBuffFinished[7];
                    byte_2_union.U8_Buff[1] = usbStr.receiveBuffFinished[8];
                    //保障转向的实时性，一得到转向信息就开始转向
                    SERVO_SetPwmValueCorrect(byte_2_union.U16);
                    //将速度和方向信息进行存储
                    icarStr.SpeedSet = byte_4_union.Float;         //速度
                    icarStr.ServoPwmSet = byte_2_union.U16;        //方向
                }

                //上位机连接通信成功
                if(!usbStr.connected)
                {
                    //蜂鸣器示意连接成功
                    Buzzer_Enable(BuzzerOk);
                    usbStr.connected = true;
                }
                //未掉线，掉线计数清零
                usbStr.counterDrop = 0;
            }
            else
            {
                //如果校验错误，翻转电平
                gpio_toggle_level(P21_5);
            }

            //清除接收控制信息，等待下一次信息的接收
            usbStr.receiveIndex = 0;
            usbStr.receiveStart = false;
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       初始阶段调车时，蓝牙调试，对应的通信接口
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void uart0_rx_interrupt_handler(void)
{
    //定义一个变量来接收数据，并处理
    uint8 temp = 0;
    //接收数据查询式,有数据会返回TRUE,没有数据会返回FALSE
    if(uart_query_byte(bluetooth_using_uart, &temp))
    {
        //如果接收到数据为帧头，并且还没还是接收，进行接下来的接收
        if(temp == DATA_PEAK_HEAD && !Bluetooth_data.receiveStart)
        {
            //切换为开始接收状态
            Bluetooth_data.receiveStart = true;
            //写入数据
            Bluetooth_data.receiveBuff[0] = temp;
            //进入下一个接收序列
            Bluetooth_data.receiveIndex = 1;
            //清零校验位
            Bluetooth_data.data_verity = 0;
            //开始接收数据
            Bluetooth_data.receiveFinished = false;
        }
        else if(Bluetooth_data.receiveStart && Bluetooth_data.receiveIndex < UART_FRAME_LEN - 2)
        {
            //写入接收到的数据
            Bluetooth_data.receiveBuff[Bluetooth_data.receiveIndex] = temp;
            //序列号加1
            Bluetooth_data.receiveIndex++;
            //进行和校验数据加和
            Bluetooth_data.data_verity += temp;
        }
        else if(Bluetooth_data.receiveIndex == UART_FRAME_LEN - 2)
        {
            //写入接收到的数据
            Bluetooth_data.receiveBuff[Bluetooth_data.receiveIndex] = temp;
            //序列号加一
            Bluetooth_data.receiveIndex++;

            //进行和校验
            if(Bluetooth_data.data_verity != temp)
            {
                //数据出现错误
                Bluetooth_data.receiveIndex = 0;
                Bluetooth_data.receiveStart = FALSE;
                //返回值-1
                uart_write_string(bluetooth_using_uart, "-1");
            }
        }
        else if(Bluetooth_data.receiveIndex == UART_FRAME_LEN - 1 && Bluetooth_data.receiveStart && temp == DATA_PEAK_TAIL)
        {
            //写入接收到的数据
            Bluetooth_data.receiveBuff[Bluetooth_data.receiveIndex] = temp;
            //序列号加一
            Bluetooth_data.receiveIndex++;
            //完成数据接收，并且校验通过
            Bluetooth_data.receiveFinished = TRUE;
        }

        //接收完数据帧，进行数据处理
        if(Bluetooth_data.receiveIndex >= UART_FRAME_LEN)
        {
            //校验成功，数据正确，拷贝数据到receiveBuffFinished，并改变接收数据状态
            memcpy(Bluetooth_data.receiveBuffFinished,Bluetooth_data.receiveBuff,UART_FRAME_LEN);
            Bluetooth_data.copyready = true;

            //接收状态清除
            Bluetooth_data.receiveStart = FALSE;
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口2接受中断函数向量接口
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    // 开启中断嵌套
    interrupt_global_enable(0);
    // 打开接收串口2
    IfxAsclin_Asc_isrReceive(&uart2_handle);
    // 串口接收处理
    uart2_rx_interrupt_handler();
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口0接受中断函数向量接口
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    // 开启中断嵌套
    interrupt_global_enable(0);
    // 打开接收串口0
    IfxAsclin_Asc_isrReceive(&uart0_handle);
    // 串口接收处理
    uart0_rx_interrupt_handler();
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       Edgeboard通信监测软件线程控制器
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void USB_Edgeboard_Timr(void)
{
    //Edgeboard通信掉线检测
    if(usbStr.connected)
    {
        //如果为连接状态，点灯，说明为连接
        gpio_low(P20_9);
        //先前连接成功了，为连接状态。如果一直有通信，会在中断中不断清零掉线计数器
        usbStr.counterDrop++;
        //如果掉线计数器时长超过3s，说明没有了通信，判断为掉线，改变相关的状态
        if(usbStr.counterDrop > 3000)
        {
            //将连接状态改为掉线
            usbStr.connected = false;
            //软件自检掉线
            usbStr.inspectorEnable = false;
        }
        //如果使用软件进行自检，进行下列操作
        if(usbStr.inspectorEnable)
        {
            usbStr.counterSend++;
        }
    }
    else
    {
        //灭灯，说明连接断开
        gpio_high(P20_9);
        //掉线之后清除pid的积累误差
        PID_clear(&car_speed_pid);
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       无线串口线程计数器
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void Wireless_Timer(void)
{
    // 定义静态变量计数器
    static uint8_t num;
    num++;

    // 每5ms无线串口或蓝牙发送一次数据
    if(num >= 5)
    {
        Wireless_data.Flag_Wireless = true;
//        Bluetooth_data.Flag_Bluetooth = true;
        num = 0;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       Edgeboard通信处理函数
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void USB_Edgeboard_Handle(void)
{
    //数据接收成功
    if(usbStr.receiveFinished)
    {
        //开始数据处理，改变接收状态，进行下一组数据接收
        usbStr.receiveFinished = false;
        //定义联合体来进行数据处理
        Byte2_Union byte2_union;
        Byte4_Union byte4_union;
        //读数据，读数据的地址第七位为1-> 地址 & 0x80 -> xxxx xxxx & 1000 0000 ->如果地址的最高位为1，与之后为非0，即为真；如果地址最高位为0，与之后值为0，即为假。
        if(usbStr.receiveBuffFinished[1] & 0x80)
        {
            //读取接收帧的地址信息，并将地址最高位置1
            uint8 Addr = (uint8)(usbStr.receiveBuffFinished[1] & 0x7F);
            //对应地址信息的处理
            switch(Addr)
            {
                //电池信息
                case USB_ADDR_BATTERY:
                    break;
                //舵机阈值
                case USB_ADDR_SERVOTHRESHOLD:
                    break;
            }
        }
        else
        {
            //根据地址信息，写数据到下位机
            switch(usbStr.receiveBuffFinished[1])
            {
                //舵机阈值
                case USB_ADDR_SERVOTHRESHOLD:
                    //进行uint16类型的数据处理
                    if(usbStr.receiveBuffFinished[3] == 1)
                    {
                        //左转阈值
                        byte2_union.U8_Buff[0] = usbStr.receiveBuffFinished[4];
                        byte2_union.U8_Buff[1] = usbStr.receiveBuffFinished[5];
                        servoStr.thresholdLeft = byte2_union.U16;
                        //下达flash存储命令
                        flashSaveEnable = true;
                        //蜂鸣器提示
                        Buzzer_Enable(BuzzerDing);
                    }
                    else if(usbStr.receiveBuffFinished[3] == 2)
                    {
                        //右转阈值
                        byte2_union.U8_Buff[0] = usbStr.receiveBuffFinished[4];
                        byte2_union.U8_Buff[1] = usbStr.receiveBuffFinished[5];
                        servoStr.thresholdRight = byte2_union.U16;
                        //下达flash存储命令
                        flashSaveEnable = true;
                        //蜂鸣器提示
                        Buzzer_Enable(BuzzerDing);
                    }
                    else if(usbStr.receiveBuffFinished[3] == 3)     //中值
                    {
                        //中值
                        byte2_union.U8_Buff[0] = usbStr.receiveBuffFinished[4];
                        byte2_union.U8_Buff[1] = usbStr.receiveBuffFinished[5];
                        servoStr.thresholdMiddle = byte2_union.U16;
                        //下达flash存储命令
                        flashSaveEnable = true;
                        //蜂鸣器提示
                        Buzzer_Enable(BuzzerDing);
                    }
                    break;
                //PID参数设置
                case USB_ADDR_PID_INFORMATION:
                    // speed-loop-kp参数值
                    for(int i=0;i<4;i++)
                        byte4_union.U8_Buff[i] = usbStr.receiveBuffFinished[3+i];
                    icarStr.speed_loop_Kp = byte4_union.Float;
                    // speed-loop-ki参数值
                    for(int i=0;i<4;i++)
                        byte4_union.U8_Buff[i] = usbStr.receiveBuffFinished[7+i];
                    icarStr.speed_loop_Ki = byte4_union.Float;
                    // speed-loop-kd参数值
                    for(int i=0;i<4;i++)
                        byte4_union.U8_Buff[i] = usbStr.receiveBuffFinished[11+i];
                    icarStr.speed_loop_Kd = byte4_union.Float;
                    // current-loop-kp参数值
                    for(int i=0;i<4;i++)
                        byte4_union.U8_Buff[i] = usbStr.receiveBuffFinished[15+i];
                    icarStr.current_loop_Kp = byte4_union.Float;
                    // current-loop-kp参数值
                    for(int i=0;i<4;i++)
                        byte4_union.U8_Buff[i] = usbStr.receiveBuffFinished[19+i];
                    icarStr.current_loop_Ki = byte4_union.Float;
                    // current-loop-kp参数值
                    for(int i=0;i<4;i++)
                        byte4_union.U8_Buff[i] = usbStr.receiveBuffFinished[23+i];
                    icarStr.current_loop_Kd = byte4_union.Float;

                    // 根据KP值判断是否开环还是闭环,0表示开环；非0表示闭环
                    if(icarStr.speed_loop_Kp != 0)
                        motorStr.CloseLoop = true;
                    else
                        motorStr.CloseLoop = false;

                    // 写入flash
                    flashPIDEnable = true;
                    // 蜂鸣器提示
                    Buzzer_Enable(BuzzerOk);
                    break;
                //蜂鸣器音效
                case USB_ADDR_BUZZER:
                    if(usbStr.receiveBuffFinished[3] == 1)          //OK
                        Buzzer_Enable(BuzzerOk);
                    else if(usbStr.receiveBuffFinished[3] == 2)     //Warnning
                        Buzzer_Enable(BuzzerWarnning);
                    else if(usbStr.receiveBuffFinished[3] == 3)     //Finish
                        Buzzer_Enable(BuzzerFinish);
                    else if(usbStr.receiveBuffFinished[3] == 4)     //Ding
                        Buzzer_Enable(BuzzerDing);
                    else if(usbStr.receiveBuffFinished[3] == 5)     //SystemStart
                        Buzzer_Enable(BuzzerSysStart);
                    break;
                //LED灯效
                case USB_ADDR_LIGHT:
                    //将数据赋值给联合体，进行数据处理
                    for(int i=0;i<4;i++)
                        byte4_union.U8_Buff[i] = usbStr.receiveBuffFinished[3+i];
                    break;
                //速控模式切换
                case USB_ADDR_SPEEDMODE:
                    //开环模式
                    if(usbStr.receiveBuffFinished[3] == 1)
                        motorStr.CloseLoop = false;
                    else
                        //闭环模式
                        motorStr.CloseLoop = true;
                    icarStr.SpeedSet = 0;
                    //蜂鸣器提示
                    Buzzer_Enable(BuzzerDing);
                    break;
//-----------------------------[自检软件相关]-------------------------------------------
                case USB_ADDR_INSPECTOR:                                   //自检软件心跳
                    usbStr.inspectorEnable = true;
                    break;
                case USB_ADDR_SELFCHECK:                                   //开始自检
                    //ICAR_SelfcheckControl(usbStr.receiveBuffFinished[3]);
                    break;
            }
        }
    }
//-----------------------[自检软件数据发送]-----------------------------
    if(usbStr.inspectorEnable && usbStr.connected && usbStr.counterSend > 150)//150ms
    {
        USB_Edgeboard_ServoThreshold(1);          //发送舵机阈值
        system_delay_ms(1);
        USB_Edgeboard_ServoThreshold(2);
        system_delay_ms(1);
        USB_Edgeboard_ServoThreshold(3);
        system_delay_ms(1);
        USB_Edgeboard_CarSpeed();                 //发送车速
        usbStr.counterSend = 0;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       Bluetooth通信处理函数
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void use_bluetooth_Handle(void)
{
    //定义判断是否为第一次进入处理接收信息
    static bool is_first = true;
    //定义联合体来处理信息
    Byte4_Union byte4_union;

    //如果数据接收成功
    if(Bluetooth_data.copyready)
    {
        //取出数据->开闭环选择
        Bluetooth_data.data_choice = Bluetooth_data.receiveBuffFinished[1];

        //定义联合体来转换数据
        Byte4_Union byte_4_union;
        //获取数据speed
        for(int i=0; i<4; i++)
            byte_4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[14+i];
        Bluetooth_data.data_speed = byte_4_union.Float;
        //获取数据angle
        for(int i=0; i<4; i++)
            byte_4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[18+i];
        Bluetooth_data.data_angle = byte_4_union.Float;
        //舵机实时控制
        servo_contral(Bluetooth_data.data_angle);

        if(is_first)
        {
            //改变状态，不是第一次来处理数据了，避免重复更新pid参数
            is_first = false;

            //获取电机速度环kp参数
            for(int i=0; i<4; i++)
                byte4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[2+i];
            Bluetooth_data.motor_speed_loop_Kp = byte4_union.Float;
            //获取电机速度环ki参数
            for(int i=0; i<4; i++)
                byte4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[6+i];
            Bluetooth_data.motor_speed_loop_Ki = byte4_union.Float;
            //获取电机速度环kd参数
            for(int i=0; i<4; i++)
                byte4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[10+i];
            Bluetooth_data.motor_speed_loop_Kd = byte4_union.Float;
            //获取电机电流环kp参数
            for(int i=0; i<4; i++)
                byte4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[22+i];
            Bluetooth_data.motor_current_loop_Kp = byte4_union.Float;
            //获取电机电流环ki参数
            for(int i=0; i<4; i++)
                byte4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[26+i];
            Bluetooth_data.motor_current_loop_Ki = byte4_union.Float;
            //获取电机电流环kd参数
            for(int i=0; i<4; i++)
                byte4_union.U8_Buff[i] = Bluetooth_data.receiveBuffFinished[30+i];
            Bluetooth_data.motor_current_loop_Kd = byte4_union.Float;

            //等待数据接收完成，点灯，说明数据接收完成
            gpio_init(P20_9, GPO, 0, GPO_PUSH_PULL);
            //获取pid参数
            float pid_data[6] = {Bluetooth_data.motor_speed_loop_Kp, Bluetooth_data.motor_speed_loop_Ki,
                                 Bluetooth_data.motor_speed_loop_Kd, Bluetooth_data.motor_current_loop_Kp,
                                 Bluetooth_data.motor_current_loop_Ki, Bluetooth_data.motor_current_loop_Kd};
            // 写入pid参数到flash中
            my_flash_write_pid(pid_data);
            // 初始化speed_loop_pid参数
            float speed_loop_pid_buff_bluetooth[3] = {Bluetooth_data.motor_speed_loop_Kp, Bluetooth_data.motor_speed_loop_Ki, Bluetooth_data.motor_speed_loop_Kd};
            PID_Init(&car_speed_pid, PID_POSITION, speed_loop_pid_buff_bluetooth, SPEED_LOOP_MAX_OUT, SPEED_LOOP_P_MAX_OUT);
            // 初始化current_loop_pid参数
            float current_loop_pid_buff_bluetooth[3] = {Bluetooth_data.motor_current_loop_Kp, Bluetooth_data.motor_current_loop_Ki, Bluetooth_data.motor_current_loop_Kd};
            PID_Init(&car_current_pid, PID_POSITION, current_loop_pid_buff_bluetooth, CURRENT_LOOP_MAX_OUT, CURRENT_LOOP_P_MAX_OUT);

            // 蜂鸣器提示
            Buzzer_Enable(BuzzerSysStart);
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       无线串口或蓝牙定时发送
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void Wireless_Handle(void)
{
    // 无线串口定时发送数据
    if(Wireless_data.Flag_Wireless == true)
    {
        Wireless_Uart_Send((int32_t)(motorStr.Set_current_motor*100000),
                           (int32_t)(motorStr.Current_motor_After_filter*100000),
                           (int32_t)(icarStr.speed_set*100000),
                           (int32_t)(icarStr.SpeedFeedback*100000));
        Wireless_data.Flag_Wireless = false;
    }

//    // 蓝牙定时发送数据
//    if(Bluetooth_data.Flag_Bluetooth == true)
//    {
//        Bluetooth_Send("%f%f%f", icarStr.SpeedSet, icarStr.SpeedFeedback, car_speed_pid.out);
//        Bluetooth_data.Flag_Bluetooth = false;
//    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       Edgeboard发送按键信号
// 参数说明       time: 按键时长
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void USB_Edgeboard_TransmitKey(uint16 time)
{
    //整理发送的数据
    uint8 check = 0;
    uint8 buff[8];
    Byte2_Union byte2_union;

    //帧头
    buff[0] = 0x42;
    //地址
    buff[1] = USB_ADDR_KEYINPUT;
    //帧长
    buff[2] = 0x06;

    //转换按键时长，并写入数据
    byte2_union.U16 = time;
    buff[3] = byte2_union.U8_Buff[0];
    buff[4] = byte2_union.U8_Buff[1];

    //进行和校验
    for(int i=0;i<5;i++)
        check += buff[i];
    //写入和校验数据
    buff[5] = check;
    //发送数据
    uart_write_buffer(eb_using_uart, buff, 8);
}


//================================[UNIT-智能汽车自检软件通信内容]==================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介       发送舵机阈值
// 参数说明       chanel: 1/左转阈值，2/右转阈值，3/中值
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void USB_Edgeboard_ServoThreshold(uint8 chanel)
{
    //超出通道选择，退出该函数
    if(chanel<1 || chanel>3)
        return;

    //整理发送的数据
    Byte2_Union byte2_union;
    uint8 check = 0;
    uint8 buff[9];
    //帧头
    buff[0] = 0x42;
    //地址
    buff[1] = USB_ADDR_SERVOTHRESHOLD;
    //帧长
    buff[2] = 0x07;
    //通道
    buff[3] = chanel;

    //选择通道，并写入发送信息
    switch(chanel)
    {
        case 1:
        {
            byte2_union.U16 = servoStr.thresholdLeft;
            buff[4] = byte2_union.U8_Buff[0];
            buff[5] = byte2_union.U8_Buff[1];
            break;
        }
        case 2:
        {
            byte2_union.U16 = servoStr.thresholdRight;
            buff[4] = byte2_union.U8_Buff[0];
            buff[5] = byte2_union.U8_Buff[1];
            break;
        }
        case 3:
        {
            byte2_union.U16 = servoStr.thresholdMiddle;
            buff[4] = byte2_union.U8_Buff[0];
            buff[5] = byte2_union.U8_Buff[1];
            break;
        }
    }

    //和校验信息
    for(int i=0;i<6;i++)
        check += buff[i];
    //写入和校验数据
    buff[6] = check;
    //发送数据
    uart_write_buffer(eb_using_uart, buff, 9);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       发送小车速度
// 参数说明       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void USB_Edgeboard_CarSpeed(void)
{
    //整理发送的数据
    Byte4_Union byte4_union;
    uint8_t check = 0;
    uint8_t buff[8];

    //帧头
    buff[0] = 0x42;
    //地址
    buff[1] = USB_ADDR_SPEEDBACK;
    //帧长
    buff[2] = 0x08;

    //发送的速度信息转换
    byte4_union.Float = icarStr.SpeedFeedback;
    buff[3] = byte4_union.U8_Buff[0];
    buff[4] = byte4_union.U8_Buff[1];
    buff[5] = byte4_union.U8_Buff[2];
    buff[6] = byte4_union.U8_Buff[3];

    //校验和信息
    for(int i=0;i<7;i++)
        check += buff[i];
    //写入和校验数据
    buff[7] = check;
    //发送数据
    uart_write_buffer(eb_using_uart, buff, 8);
}


//==============================[向上位机发送通信内容]====================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     向上位机发送数据
// 参数说明     senddata  将要发送的所有数据写入
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void senddata_to_upper(uint8_t addr,float senddata)
{
    //整理发送的数据
    Byte4_Union byte4_union;
    uint8_t check = 0;
    uint8_t buff[8];

    //帧头
    buff[0] = 0x42;
    //地址
    buff[1] = addr;
    //帧长
    buff[2] = 0x08;

    //发送的速度信息转换
    byte4_union.Float = senddata;
    buff[3] = byte4_union.U8_Buff[0];
    buff[4] = byte4_union.U8_Buff[1];
    buff[5] = byte4_union.U8_Buff[2];
    buff[6] = byte4_union.U8_Buff[3];

    //校验和信息
    for(int i=0;i<7;i++)
        check += buff[i];
    //写入和校验数据
    buff[7] = check;
    //发送数据
    uart_write_buffer(eb_using_uart, buff, 8);
}




