/*
 * motor.h
 *
 *  Created on: 2023��3��20��
 *      Author: wzl
 */
#ifndef CODE_MOTOR_MOTOR_H_
#define CODE_MOTOR_MOTOR_H_

//����ͷ�ļ�
#include "cpu0_main.h"
#include "uart/uart.h"

//�궨����Ʒ�ʽ,0Ϊ�ߵ͵�ƽ��������װ��1Ϊ��·pwm��������ת
#define motor_control_measure   0

//���pwm������ź궨��
#if motor_control_measure
    #define pwm_positive    51      //ʹ�õ�ATOM1_CH0_P21_2
    #define pwm_nagetive    70      //ʹ�õ�ATOM1_CH0_P21_4
#else
    #define pwm_rotation    70      //ʹ�õ�ATOM1_CH0_P21_4
    #define pwm_control     674     //ʹ�õ�IO���ſ��Ʒ���21_2
#endif

//�������������ź궨��
#define encoder_pin_CH1   1         //33.7
#define encoder_pin_CH2   1         //33.6
//ʹ�ö�ʱ���궨��
#define USING_TIMER       0         //tim2

//������ƺ궨��
#define CAR_MAX_SPEED   9500        //������ռ�ձȣ���mos��
#define CAR_IMAX_OUT    4000

//�궨��pi
#define PI_MOTOR                    3.1415926535898f
//�����������T��5ms
#define MOTOR_CONTROL_CYCLE         0.005f

//���������ƽṹ��
typedef struct
{
    float ReductionRatio;                       //������ٱ�
    float EncoderLine;                          //����������
    int16 EncoderValue;                         //������ʵʱ�ٶ�,��ʱ����ֵ
    float DiameterWheel;                        //����ֱ����mm
    uint8_t CloseLoop;                          //����ģʽ
    uint16_t Counter;                           //�̼߳�����
}MotorStruct;

//�����ṹ��
extern MotorStruct motorStr;

//====================================================MOTOR ��������====================================================
void motor_init                 (void);
void motor_SetPwmValue          (int16_t pwm);
void motor_ControlLoop          (void);
//====================================================MOTOR ��������====================================================


#endif /* CODE_MOTOR_MOTOR_H_ */
