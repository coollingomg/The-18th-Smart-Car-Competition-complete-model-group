#include "IfxGtm_Atom_Pwm.h"
#include "ifxGtm_PinMap.h"
#include "zf_common_debug.h"
#include "zf_driver_pwm.h"

#define CMU_CLK_FREQ           20000000.0f                       // CMU时钟频率

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     获取端口参数
//  返回参数     IfxGtm_Atom_ToutMap
//  使用示例     get_pwm_pin(ATOM0_CH0_P00_0);
//  备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static IfxGtm_Atom_ToutMap* get_pwm_pin (pwm_channel_enum atom_pin)
{
    IfxGtm_Atom_ToutMap* pwm_pwm_pin_config;

    switch(atom_pin)
    {
        case ATOM0_CH0_P00_0: pwm_pwm_pin_config = &IfxGtm_ATOM0_0_TOUT9_P00_0_OUT;     break;
        case ATOM0_CH0_P02_0: pwm_pwm_pin_config = &IfxGtm_ATOM0_0_TOUT0_P02_0_OUT;     break;
        case ATOM0_CH0_P02_8: pwm_pwm_pin_config = &IfxGtm_ATOM0_0_TOUT8_P02_8_OUT;     break;
        case ATOM0_CH0_P14_5: pwm_pwm_pin_config = &IfxGtm_ATOM0_0_TOUT85_P14_5_OUT;    break;
        case ATOM0_CH0_P21_2: pwm_pwm_pin_config = &IfxGtm_ATOM0_0_TOUT53_P21_2_OUT;    break;
        case ATOM0_CH0_P22_1: pwm_pwm_pin_config = &IfxGtm_ATOM0_0_TOUT48_P22_1_OUT;    break;

        case ATOM0_CH1_P00_1: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT10_P00_1_OUT;    break;
        case ATOM0_CH1_P00_2: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT11_P00_2_OUT;    break;
        case ATOM0_CH1_P02_1: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT1_P02_1_OUT;     break;
        case ATOM0_CH1_P10_1: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT103_P10_1_OUT;   break;
        case ATOM0_CH1_P14_4: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT84_P14_4_OUT;    break;
        case ATOM0_CH1_P21_3: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT54_P21_3_OUT;    break;
        case ATOM0_CH1_P22_0: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT47_P22_0_OUT;    break;
        case ATOM0_CH1_P33_9: pwm_pwm_pin_config = &IfxGtm_ATOM0_1_TOUT31_P33_9_OUT;    break;

        case ATOM0_CH2_P00_3: pwm_pwm_pin_config = &IfxGtm_ATOM0_2_TOUT12_P00_3_OUT;    break;
        case ATOM0_CH2_P02_2: pwm_pwm_pin_config = &IfxGtm_ATOM0_2_TOUT2_P02_2_OUT;     break;
        case ATOM0_CH2_P10_2: pwm_pwm_pin_config = &IfxGtm_ATOM0_2_TOUT104_P10_2_OUT;   break;
        case ATOM0_CH2_P10_5: pwm_pwm_pin_config = &IfxGtm_ATOM0_2_TOUT107_P10_5_OUT;   break;
        case ATOM0_CH2_P14_3: pwm_pwm_pin_config = &IfxGtm_ATOM0_2_TOUT83_P14_3_OUT;    break;
        case ATOM0_CH2_P21_4: pwm_pwm_pin_config = &IfxGtm_ATOM0_2_TOUT55_P21_4_OUT;    break;
        case ATOM0_CH2_P33_11:pwm_pwm_pin_config = &IfxGtm_ATOM0_2_TOUT33_P33_11_OUT;   break;

        case ATOM0_CH3_P00_4: pwm_pwm_pin_config = &IfxGtm_ATOM0_3_TOUT13_P00_4_OUT;    break;
        case ATOM0_CH3_P02_3: pwm_pwm_pin_config = &IfxGtm_ATOM0_3_TOUT3_P02_3_OUT;     break;
        case ATOM0_CH3_P10_3: pwm_pwm_pin_config = &IfxGtm_ATOM0_3_TOUT105_P10_3_OUT;   break;
        case ATOM0_CH3_P10_6: pwm_pwm_pin_config = &IfxGtm_ATOM0_3_TOUT108_P10_6_OUT;   break;
        case ATOM0_CH3_P14_2: pwm_pwm_pin_config = &IfxGtm_ATOM0_3_TOUT82_P14_2_OUT;    break;
        case ATOM0_CH3_P21_5: pwm_pwm_pin_config = &IfxGtm_ATOM0_3_TOUT56_P21_5_OUT;    break;
        case ATOM0_CH3_P22_2: pwm_pwm_pin_config = &IfxGtm_ATOM0_3_TOUT49_P22_2_OUT;    break;

        case ATOM0_CH4_P00_5: pwm_pwm_pin_config = &IfxGtm_ATOM0_4_TOUT14_P00_5_OUT;    break;
        case ATOM0_CH4_P02_4: pwm_pwm_pin_config = &IfxGtm_ATOM0_4_TOUT4_P02_4_OUT;     break;
        case ATOM0_CH4_P14_1: pwm_pwm_pin_config = &IfxGtm_ATOM0_4_TOUT81_P14_1_OUT;    break;
        case ATOM0_CH4_P20_3: pwm_pwm_pin_config = &IfxGtm_ATOM0_4_TOUT61_P20_3_OUT;    break;
        case ATOM0_CH4_P21_6: pwm_pwm_pin_config = &IfxGtm_ATOM0_4_TOUT57_P21_6_OUT;    break;
        case ATOM0_CH4_P22_3: pwm_pwm_pin_config = &IfxGtm_ATOM0_4_TOUT50_P22_3_OUT;    break;

        case ATOM0_CH5_P00_6: pwm_pwm_pin_config = &IfxGtm_ATOM0_5_TOUT15_P00_6_OUT;    break;
        case ATOM0_CH5_P02_5: pwm_pwm_pin_config = &IfxGtm_ATOM0_5_TOUT5_P02_5_OUT;     break;
        case ATOM0_CH5_P21_7: pwm_pwm_pin_config = &IfxGtm_ATOM0_5_TOUT58_P21_7_OUT;    break;
        case ATOM0_CH5_P32_4: pwm_pwm_pin_config = &IfxGtm_ATOM0_5_TOUT40_P32_4_OUT;    break;

        case ATOM0_CH6_P00_7: pwm_pwm_pin_config = &IfxGtm_ATOM0_6_TOUT16_P00_7_OUT;    break;
        case ATOM0_CH6_P02_6: pwm_pwm_pin_config = &IfxGtm_ATOM0_6_TOUT6_P02_6_OUT;     break;
        case ATOM0_CH6_P20_0: pwm_pwm_pin_config = &IfxGtm_ATOM0_6_TOUT59_P20_0_OUT;    break;
        case ATOM0_CH6_P23_1: pwm_pwm_pin_config = &IfxGtm_ATOM0_6_TOUT42_P23_1_OUT;    break;

        case ATOM0_CH7_P00_8: pwm_pwm_pin_config = &IfxGtm_ATOM0_7_TOUT17_P00_8_OUT;    break;
        case ATOM0_CH7_P02_7: pwm_pwm_pin_config = &IfxGtm_ATOM0_7_TOUT7_P02_7_OUT;     break;
        case ATOM0_CH7_P20_8: pwm_pwm_pin_config = &IfxGtm_ATOM0_7_TOUT64_P20_8_OUT;    break;

        case ATOM1_CH0_P00_0: pwm_pwm_pin_config = &IfxGtm_ATOM1_0_TOUT9_P00_0_OUT;     break;
        case ATOM1_CH0_P02_0: pwm_pwm_pin_config = &IfxGtm_ATOM1_0_TOUT0_P02_0_OUT;     break;
        case ATOM1_CH0_P02_8: pwm_pwm_pin_config = &IfxGtm_ATOM1_0_TOUT8_P02_8_OUT;     break;
        case ATOM1_CH0_P15_5: pwm_pwm_pin_config = &IfxGtm_ATOM1_0_TOUT76_P15_5_OUT;    break;
        case ATOM1_CH0_P15_6: pwm_pwm_pin_config = &IfxGtm_ATOM1_0_TOUT77_P15_6_OUT;    break;
        case ATOM1_CH0_P20_12:pwm_pwm_pin_config = &IfxGtm_ATOM1_0_TOUT68_P20_12_OUT;   break;
        case ATOM1_CH0_P21_2: pwm_pwm_pin_config = &IfxGtm_ATOM1_0_TOUT53_P21_2_OUT;    break;
        case ATOM1_CH0_P22_1: pwm_pwm_pin_config = &IfxGtm_ATOM1_0_TOUT48_P22_1_OUT;    break;

        case ATOM1_CH1_P00_1: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT10_P00_1_OUT;    break;
        case ATOM1_CH1_P00_2: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT11_P00_2_OUT;    break;
        case ATOM1_CH1_P02_1: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT1_P02_1_OUT;     break;
        case ATOM1_CH1_P10_1: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT103_P10_1_OUT;   break;
        case ATOM1_CH1_P14_6: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT86_P14_6_OUT;    break;
        case ATOM1_CH1_P15_7: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT78_P15_7_OUT;    break;
        case ATOM1_CH1_P15_8: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT79_P15_8_OUT;    break;
        case ATOM1_CH1_P20_13:pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT69_P20_13_OUT;   break;
        case ATOM1_CH1_P21_3: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT54_P21_3_OUT;    break;
        case ATOM1_CH1_P22_0: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT47_P22_0_OUT;    break;
        case ATOM1_CH1_P33_9: pwm_pwm_pin_config = &IfxGtm_ATOM1_1_TOUT31_P33_9_OUT;    break;

        case ATOM1_CH2_P00_3: pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT12_P00_3_OUT;    break;
        case ATOM1_CH2_P02_2: pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT2_P02_2_OUT;     break;
        case ATOM1_CH2_P10_2: pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT104_P10_2_OUT;   break;
        case ATOM1_CH2_P10_5: pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT107_P10_5_OUT;   break;
        case ATOM1_CH2_P14_0: pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT80_P14_0_OUT;    break;
        case ATOM1_CH2_P20_14:pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT70_P20_14_OUT;   break;
        case ATOM1_CH2_P21_4: pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT55_P21_4_OUT;    break;
        case ATOM1_CH2_P33_11:pwm_pwm_pin_config = &IfxGtm_ATOM1_2_TOUT33_P33_11_OUT;   break;

        case ATOM1_CH3_P00_4: pwm_pwm_pin_config = &IfxGtm_ATOM1_3_TOUT13_P00_4_OUT;    break;
        case ATOM1_CH3_P02_3: pwm_pwm_pin_config = &IfxGtm_ATOM1_3_TOUT3_P02_3_OUT;     break;
        case ATOM1_CH3_P10_3: pwm_pwm_pin_config = &IfxGtm_ATOM1_3_TOUT105_P10_3_OUT;   break;
        case ATOM1_CH3_P10_6: pwm_pwm_pin_config = &IfxGtm_ATOM1_3_TOUT108_P10_6_OUT;   break;
        case ATOM1_CH3_P15_0: pwm_pwm_pin_config = &IfxGtm_ATOM1_3_TOUT71_P15_0_OUT;    break;
        case ATOM1_CH3_P21_5: pwm_pwm_pin_config = &IfxGtm_ATOM1_3_TOUT56_P21_5_OUT;    break;
        case ATOM1_CH3_P22_2: pwm_pwm_pin_config = &IfxGtm_ATOM1_3_TOUT49_P22_2_OUT;    break;

        case ATOM1_CH4_P00_5: pwm_pwm_pin_config = &IfxGtm_ATOM1_4_TOUT14_P00_5_OUT;    break;
        case ATOM1_CH4_P02_4: pwm_pwm_pin_config = &IfxGtm_ATOM1_4_TOUT4_P02_4_OUT;     break;
        case ATOM1_CH4_P15_1: pwm_pwm_pin_config = &IfxGtm_ATOM1_4_TOUT72_P15_1_OUT;    break;
        case ATOM1_CH4_P20_3: pwm_pwm_pin_config = &IfxGtm_ATOM1_4_TOUT61_P20_3_OUT;    break;
        case ATOM1_CH4_P21_6: pwm_pwm_pin_config = &IfxGtm_ATOM1_4_TOUT57_P21_6_OUT;    break;
        case ATOM1_CH4_P22_3: pwm_pwm_pin_config = &IfxGtm_ATOM1_4_TOUT50_P22_3_OUT;    break;

        case ATOM1_CH5_P00_6: pwm_pwm_pin_config = &IfxGtm_ATOM1_5_TOUT15_P00_6_OUT;    break;
        case ATOM1_CH5_P02_5: pwm_pwm_pin_config = &IfxGtm_ATOM1_5_TOUT5_P02_5_OUT;     break;
        case ATOM1_CH5_P15_2: pwm_pwm_pin_config = &IfxGtm_ATOM1_5_TOUT73_P15_2_OUT;    break;
        case ATOM1_CH5_P20_9: pwm_pwm_pin_config = &IfxGtm_ATOM1_5_TOUT65_P20_9_OUT;    break;
        case ATOM1_CH5_P21_7: pwm_pwm_pin_config = &IfxGtm_ATOM1_5_TOUT58_P21_7_OUT;    break;
        case ATOM1_CH5_P32_4: pwm_pwm_pin_config = &IfxGtm_ATOM1_5_TOUT40_P32_4_OUT;    break;

        case ATOM1_CH6_P00_7: pwm_pwm_pin_config = &IfxGtm_ATOM1_6_TOUT16_P00_7_OUT;    break;
        case ATOM1_CH6_P02_6: pwm_pwm_pin_config = &IfxGtm_ATOM1_6_TOUT6_P02_6_OUT;     break;
        case ATOM1_CH6_P15_3: pwm_pwm_pin_config = &IfxGtm_ATOM1_6_TOUT74_P15_3_OUT;    break;
        case ATOM1_CH6_P20_0: pwm_pwm_pin_config = &IfxGtm_ATOM1_6_TOUT59_P20_0_OUT;    break;
        case ATOM1_CH6_P20_10:pwm_pwm_pin_config = &IfxGtm_ATOM1_6_TOUT66_P20_10_OUT;   break;
        case ATOM1_CH6_P23_1: pwm_pwm_pin_config = &IfxGtm_ATOM1_6_TOUT42_P23_1_OUT;    break;

        case ATOM1_CH7_P00_8: pwm_pwm_pin_config = &IfxGtm_ATOM1_7_TOUT17_P00_8_OUT;    break;
        case ATOM1_CH7_P02_7: pwm_pwm_pin_config = &IfxGtm_ATOM1_7_TOUT7_P02_7_OUT;     break;
        case ATOM1_CH7_P15_4: pwm_pwm_pin_config = &IfxGtm_ATOM1_7_TOUT75_P15_4_OUT;    break;
        case ATOM1_CH7_P20_11:pwm_pwm_pin_config = &IfxGtm_ATOM1_7_TOUT67_P20_11_OUT;   break;

        case ATOM2_CH0_P00_9: pwm_pwm_pin_config = &IfxGtm_ATOM2_0_TOUT18_P00_9_OUT;    break;
        case ATOM2_CH0_P13_3: pwm_pwm_pin_config = &IfxGtm_ATOM2_0_TOUT94_P13_3_OUT;    break;
        case ATOM2_CH0_P20_12:pwm_pwm_pin_config = &IfxGtm_ATOM2_0_TOUT68_P20_12_OUT;   break;
        case ATOM2_CH0_P33_4: pwm_pwm_pin_config = &IfxGtm_ATOM2_0_TOUT26_P33_4_OUT;    break;
        case ATOM2_CH0_P33_10:pwm_pwm_pin_config = &IfxGtm_ATOM2_0_TOUT32_P33_10_OUT;   break;

        case ATOM2_CH1_P11_2: pwm_pwm_pin_config = &IfxGtm_ATOM2_1_TOUT95_P11_2_OUT;    break;
        case ATOM2_CH1_P20_13:pwm_pwm_pin_config = &IfxGtm_ATOM2_1_TOUT69_P20_13_OUT;   break;
        case ATOM2_CH1_P33_5: pwm_pwm_pin_config = &IfxGtm_ATOM2_1_TOUT27_P33_5_OUT;    break;

        case ATOM2_CH2_P11_3: pwm_pwm_pin_config = &IfxGtm_ATOM2_2_TOUT96_P11_3_OUT;    break;
        case ATOM2_CH2_P20_14:pwm_pwm_pin_config = &IfxGtm_ATOM2_2_TOUT70_P20_14_OUT;   break;
        case ATOM2_CH2_P33_6: pwm_pwm_pin_config = &IfxGtm_ATOM2_2_TOUT28_P33_6_OUT;    break;

        case ATOM2_CH3_P00_12:pwm_pwm_pin_config = &IfxGtm_ATOM2_3_TOUT21_P00_12_OUT;   break;
        case ATOM2_CH3_P11_6: pwm_pwm_pin_config = &IfxGtm_ATOM2_3_TOUT97_P11_6_OUT;    break;
        case ATOM2_CH3_P15_0: pwm_pwm_pin_config = &IfxGtm_ATOM2_3_TOUT71_P15_0_OUT;    break;
        case ATOM2_CH3_P33_7: pwm_pwm_pin_config = &IfxGtm_ATOM2_3_TOUT29_P33_7_OUT;    break;

        case ATOM2_CH4_P11_9: pwm_pwm_pin_config = &IfxGtm_ATOM2_4_TOUT98_P11_9_OUT;    break;
        case ATOM2_CH4_P15_1: pwm_pwm_pin_config = &IfxGtm_ATOM2_4_TOUT72_P15_1_OUT;    break;
        case ATOM2_CH4_P33_8: pwm_pwm_pin_config = &IfxGtm_ATOM2_4_TOUT30_P33_8_OUT;    break;
        case ATOM2_CH4_P33_12:pwm_pwm_pin_config = &IfxGtm_ATOM2_4_TOUT34_P33_12_OUT;   break;

        case ATOM2_CH5_P11_10:pwm_pwm_pin_config = &IfxGtm_ATOM2_5_TOUT99_P11_10_OUT;   break;
        case ATOM2_CH5_P13_0: pwm_pwm_pin_config = &IfxGtm_ATOM2_5_TOUT91_P13_0_OUT;    break;
        case ATOM2_CH5_P15_2: pwm_pwm_pin_config = &IfxGtm_ATOM2_5_TOUT73_P15_2_OUT;    break;
        case ATOM2_CH5_P20_9: pwm_pwm_pin_config = &IfxGtm_ATOM2_5_TOUT65_P20_9_OUT;    break;
        case ATOM2_CH5_P33_13:pwm_pwm_pin_config = &IfxGtm_ATOM2_5_TOUT35_P33_13_OUT;   break;

        case ATOM2_CH6_P11_11:pwm_pwm_pin_config = &IfxGtm_ATOM2_6_TOUT100_P11_11_OUT;  break;
        case ATOM2_CH6_P13_1: pwm_pwm_pin_config = &IfxGtm_ATOM2_6_TOUT92_P13_1_OUT;    break;
        case ATOM2_CH6_P15_3: pwm_pwm_pin_config = &IfxGtm_ATOM2_6_TOUT74_P15_3_OUT;    break;
        case ATOM2_CH6_P20_6: pwm_pwm_pin_config = &IfxGtm_ATOM2_6_TOUT62_P20_6_OUT;    break;
        case ATOM2_CH6_P20_10:pwm_pwm_pin_config = &IfxGtm_ATOM2_6_TOUT66_P20_10_OUT;   break;
        case ATOM2_CH6_P32_0: pwm_pwm_pin_config = &IfxGtm_ATOM2_6_TOUT36_P32_0_OUT;    break;

        case ATOM2_CH7_P11_12:pwm_pwm_pin_config = &IfxGtm_ATOM2_7_TOUT101_P11_12_OUT;  break;
        case ATOM2_CH7_P13_2: pwm_pwm_pin_config = &IfxGtm_ATOM2_7_TOUT93_P13_2_OUT;    break;
        case ATOM2_CH7_P15_4: pwm_pwm_pin_config = &IfxGtm_ATOM2_7_TOUT75_P15_4_OUT;    break;
        case ATOM2_CH7_P20_7: pwm_pwm_pin_config = &IfxGtm_ATOM2_7_TOUT63_P20_7_OUT;    break;
        case ATOM2_CH7_P20_8: pwm_pwm_pin_config = &IfxGtm_ATOM2_7_TOUT64_P20_8_OUT;    break;
        case ATOM2_CH7_P20_11:pwm_pwm_pin_config = &IfxGtm_ATOM2_7_TOUT67_P20_11_OUT;   break;

        case ATOM3_CH0_P00_9: pwm_pwm_pin_config = &IfxGtm_ATOM3_0_TOUT18_P00_9_OUT;    break;
        case ATOM3_CH0_P13_3: pwm_pwm_pin_config = &IfxGtm_ATOM3_0_TOUT94_P13_3_OUT;    break;
        case ATOM3_CH0_P14_5: pwm_pwm_pin_config = &IfxGtm_ATOM3_0_TOUT85_P14_5_OUT;    break;
        case ATOM3_CH0_P15_5: pwm_pwm_pin_config = &IfxGtm_ATOM3_0_TOUT76_P15_5_OUT;    break;
        case ATOM3_CH0_P15_6: pwm_pwm_pin_config = &IfxGtm_ATOM3_0_TOUT77_P15_6_OUT;    break;
        case ATOM3_CH0_P33_4: pwm_pwm_pin_config = &IfxGtm_ATOM3_0_TOUT26_P33_4_OUT;    break;
        case ATOM3_CH0_P33_10:pwm_pwm_pin_config = &IfxGtm_ATOM3_0_TOUT32_P33_10_OUT;   break;

        case ATOM3_CH1_P11_2: pwm_pwm_pin_config = &IfxGtm_ATOM3_1_TOUT95_P11_2_OUT;    break;
        case ATOM3_CH1_P14_4: pwm_pwm_pin_config = &IfxGtm_ATOM3_1_TOUT84_P14_4_OUT;    break;
        case ATOM3_CH1_P14_6: pwm_pwm_pin_config = &IfxGtm_ATOM3_1_TOUT86_P14_6_OUT;    break;
        case ATOM3_CH1_P15_7: pwm_pwm_pin_config = &IfxGtm_ATOM3_1_TOUT78_P15_7_OUT;    break;
        case ATOM3_CH1_P15_8: pwm_pwm_pin_config = &IfxGtm_ATOM3_1_TOUT79_P15_8_OUT;    break;
        case ATOM3_CH1_P33_5: pwm_pwm_pin_config = &IfxGtm_ATOM3_1_TOUT27_P33_5_OUT;    break;

        case ATOM3_CH2_P11_3: pwm_pwm_pin_config = &IfxGtm_ATOM3_2_TOUT96_P11_3_OUT;    break;
        case ATOM3_CH2_P14_0: pwm_pwm_pin_config = &IfxGtm_ATOM3_2_TOUT80_P14_0_OUT;    break;
        case ATOM3_CH2_P14_3: pwm_pwm_pin_config = &IfxGtm_ATOM3_2_TOUT83_P14_3_OUT;    break;
        case ATOM3_CH2_P33_6: pwm_pwm_pin_config = &IfxGtm_ATOM3_2_TOUT28_P33_6_OUT;    break;

        case ATOM3_CH3_P00_12:pwm_pwm_pin_config = &IfxGtm_ATOM3_3_TOUT21_P00_12_OUT;   break;
        case ATOM3_CH3_P11_6: pwm_pwm_pin_config = &IfxGtm_ATOM3_3_TOUT97_P11_6_OUT;    break;
        case ATOM3_CH3_P14_2: pwm_pwm_pin_config = &IfxGtm_ATOM3_3_TOUT82_P14_2_OUT;    break;
        case ATOM3_CH3_P33_7: pwm_pwm_pin_config = &IfxGtm_ATOM3_3_TOUT29_P33_7_OUT;    break;

        case ATOM3_CH4_P11_9: pwm_pwm_pin_config = &IfxGtm_ATOM3_4_TOUT98_P11_9_OUT;    break;
        case ATOM3_CH4_P14_1: pwm_pwm_pin_config = &IfxGtm_ATOM3_4_TOUT81_P14_1_OUT;    break;
        case ATOM3_CH4_P33_8: pwm_pwm_pin_config = &IfxGtm_ATOM3_4_TOUT30_P33_8_OUT;    break;
        case ATOM3_CH4_P33_12:pwm_pwm_pin_config = &IfxGtm_ATOM3_4_TOUT34_P33_12_OUT;   break;

        case ATOM3_CH5_P11_10:pwm_pwm_pin_config = &IfxGtm_ATOM3_5_TOUT99_P11_10_OUT;   break;
        case ATOM3_CH5_P13_0: pwm_pwm_pin_config = &IfxGtm_ATOM3_5_TOUT91_P13_0_OUT;    break;
        case ATOM3_CH5_P33_13:pwm_pwm_pin_config = &IfxGtm_ATOM3_5_TOUT35_P33_13_OUT;   break;

        case ATOM3_CH6_P11_11:pwm_pwm_pin_config = &IfxGtm_ATOM3_6_TOUT100_P11_11_OUT;  break;
        case ATOM3_CH6_P13_1: pwm_pwm_pin_config = &IfxGtm_ATOM3_6_TOUT92_P13_1_OUT;    break;
        case ATOM3_CH6_P20_6: pwm_pwm_pin_config = &IfxGtm_ATOM3_6_TOUT62_P20_6_OUT;    break;
        case ATOM3_CH6_P32_0: pwm_pwm_pin_config = &IfxGtm_ATOM3_6_TOUT36_P32_0_OUT;    break;

        case ATOM3_CH7_P11_12:pwm_pwm_pin_config = &IfxGtm_ATOM3_7_TOUT101_P11_12_OUT;  break;
        case ATOM3_CH7_P13_2: pwm_pwm_pin_config = &IfxGtm_ATOM3_7_TOUT93_P13_2_OUT;    break;
        case ATOM3_CH7_P20_7: pwm_pwm_pin_config = &IfxGtm_ATOM3_7_TOUT63_P20_7_OUT;    break;

        default: zf_assert(FALSE); pwm_pwm_pin_config = NULL;
    }
    return pwm_pwm_pin_config;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      关闭所有通道的PWM输出
//  返回参数      void
//  使用示例      pwm_all_channel_close();
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void pwm_all_channel_close (void)
{
    IfxGtm_Atom_Pwm_Config g_atomConfig;
    IfxGtm_Atom_Pwm_Driver g_atomDriver;

    int index,channel;

    IfxGtm_enable(&MODULE_GTM);

    if(!(MODULE_GTM.CMU.CLK_EN.U & 0x2))
    {
        IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, CMU_CLK_FREQ);
        IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);
    }
    IfxGtm_Atom_Pwm_initConfig(&g_atomConfig, &MODULE_GTM);

    for(index = 0; index < 4; index++)
    {
        for(channel = 0; channel < 8; channel++)
        {
            g_atomConfig.atom = index;
            g_atomConfig.atomChannel = channel;
            IfxGtm_Atom_Pwm_init(&g_atomDriver, &g_atomConfig);
            IfxGtm_Atom_Pwm_stop(&g_atomDriver, TRUE);
        }
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PWM占空比设置
// 参数说明     pin             选择 PWM 引脚
// 参数说明     duty            设置占空比
// 返回参数     void
// 使用示例     pwm_set_duty(ATOM0_CH7_P02_7, 5000); // 设置占空比为百分之5000/PWM_DUTY_MAX*100
// 备注信息     GTM_ATOM0_PWM_DUTY_MAX宏定义在zf_driver_pwm.h  默认为10000
//-------------------------------------------------------------------------------------------------------------------
void pwm_set_duty (pwm_channel_enum pwmch, uint32 duty)
{
    uint32 period;

    zf_assert(duty <= PWM_DUTY_MAX);    // 如果在这里出现了断言，那么说明你输入的占空比已经大于了最大占空比 PWM_DUTY_MAX 宏定义在zf_driver_pwm.h  默认为10000

    IfxGtm_Atom_ToutMap *atom_channel;
    atom_channel = get_pwm_pin(pwmch);

    period = IfxGtm_Atom_Ch_getCompareZero(&MODULE_GTM.ATOM[atom_channel->atom], atom_channel->channel);

    switch(atom_channel->atom)
    {
        case 0: duty = (uint32)((uint64)duty * period / PWM_DUTY_MAX); break;
        case 1: duty = (uint32)((uint64)duty * period / PWM_DUTY_MAX); break;
        case 2: duty = (uint32)((uint64)duty * period / PWM_DUTY_MAX); break;
        case 3: duty = (uint32)((uint64)duty * period / PWM_DUTY_MAX); break;
    }
    IfxGtm_Atom_Ch_setCompareOneShadow(&MODULE_GTM.ATOM[atom_channel->atom], atom_channel->channel, duty);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PWM 初始化
// 参数说明     pin             选择 PWM 引脚
// 参数说明     freq            设置频率 同个模块只有最后一次设置生效
// 参数说明     duty            设置占空比
// 返回参数     void
// 使用示例     pwm_init(ATOM0_CH7_P02_7, 50, 1000);   // ATOM 0模块的通道7 使用P02_7引脚输出PWM  PWM频率50HZ  占空比百分之1000/PWM_DUTY_MAX*100
// 备注信息     PWM_DUTY_MAX宏定义在zf_driver_pwm.h  默认为10000
//-------------------------------------------------------------------------------------------------------------------
void pwm_init (pwm_channel_enum pwmch, uint32 freq, uint32 duty)
{
    IfxGtm_Atom_Pwm_Config g_atomConfig;
    IfxGtm_Atom_Pwm_Driver g_atomDriver;

    IfxGtm_Atom_ToutMap *atom_channel;

    zf_assert(duty <= PWM_DUTY_MAX);    // 如果在这里出现了断言，那么说明你输入的占空比已经大于了最大占空比 PWM_DUTY_MAX 宏定义在zf_driver_pwm.h  默认为10000


    atom_channel = get_pwm_pin(pwmch);

    switch(atom_channel->atom)
    {
        case 0: IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, duty <= PWM_DUTY_MAX); break;
        case 1: IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, duty <= PWM_DUTY_MAX); break;
        case 2: IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, duty <= PWM_DUTY_MAX); break;
        case 3: IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, duty <= PWM_DUTY_MAX); break;
    }

    IfxGtm_enable(&MODULE_GTM);

    if(!(MODULE_GTM.CMU.CLK_EN.U & 0x2))
    {
        IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, CMU_CLK_FREQ);
        IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);
    }

    IfxGtm_Atom_Pwm_initConfig(&g_atomConfig, &MODULE_GTM);

    g_atomConfig.atom = atom_channel->atom;
    g_atomConfig.atomChannel = atom_channel->channel;
    g_atomConfig.period = CMU_CLK_FREQ/freq;
    g_atomConfig.pin.outputPin = atom_channel;
    g_atomConfig.synchronousUpdateEnabled = TRUE;

    switch(atom_channel->atom)
    {
        case 0: g_atomConfig.dutyCycle = (uint32)((uint64)duty * g_atomConfig.period / PWM_DUTY_MAX); break;
        case 1: g_atomConfig.dutyCycle = (uint32)((uint64)duty * g_atomConfig.period / PWM_DUTY_MAX); break;
        case 2: g_atomConfig.dutyCycle = (uint32)((uint64)duty * g_atomConfig.period / PWM_DUTY_MAX); break;
        case 3: g_atomConfig.dutyCycle = (uint32)((uint64)duty * g_atomConfig.period / PWM_DUTY_MAX); break;
    }

    IfxGtm_Atom_Pwm_init(&g_atomDriver, &g_atomConfig);
    IfxGtm_Atom_Pwm_start(&g_atomDriver, TRUE);
}


