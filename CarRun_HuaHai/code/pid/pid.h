#ifndef PID_H
#define PID_H

#include "cpu0_main.h"

enum PID_MODE
{
    PID_POSITION = 0,//λ��ʽPID
    PID_DELTA        //����ʽPID
};

typedef struct
{
    uint8_t mode_t;

    //PID ����
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������

    float set;      //�趨ֵ
    float fdb;      //ʵ��ֵ

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�
} PidTypeDef;

//�����ṹ��
extern PidTypeDef car_speed_pid;

//====================================================PID ��������====================================================
extern void PID_Init        (PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
extern float PID_Calc       (PidTypeDef *pid, float ref, float set);
extern void PID_clear       (PidTypeDef *pid);
//====================================================PID ��������====================================================


#endif
