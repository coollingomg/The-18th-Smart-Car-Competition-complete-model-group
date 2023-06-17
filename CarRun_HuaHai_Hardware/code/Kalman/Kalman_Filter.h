#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H


typedef struct 
{
    float LastP;    //�ϴι���Э���� ��ʼ��ֵΪ0.02
    float Now_P;    //��ǰ����Э���� ��ʼ��ֵΪ0
    float out;      //�������˲������ ��ʼ��ֵΪ0
    float Kg;       //���������� ��ʼ��ֵΪ0
    float Q;        //��������Э���� ��ʼ��ֵΪ0.001
    float R;        //�۲�����Э���� ��ʼ��ֵΪ0.543
}KalmanInfo;        //Kalman Filter parameter

//�����������ṹ�����
extern KalmanInfo kalman_struck;
extern KalmanInfo kalman_struck1;

void  Kalman_Filter_Init(KalmanInfo *KalmanInfo_Structure);
float Kalman_Filter_Fun(KalmanInfo *info, float new_value);

#endif
