/*
 * buzzer.h
 *
 *  Created on: 2023��4��27��
 *      Author: wzl
 */

#ifndef CODE_BUZZER_BUZZER_H_
#define CODE_BUZZER_BUZZER_H_

//����ͷ�ļ�
#include "cpu0_main.h"

//����������ṹ��
typedef struct
{
    bool Enable;                        //ʹ�ܱ�־
    uint16_t Times;                     //���д���
    uint16_t Counter;                   //������
    uint16_t Cut;                       //���ʱ��
    bool Silent;                        //�Ƿ���÷�����
}BuzzerStruct;


//��������Ч
typedef enum
{
    BuzzerOk = 0,                       //ȷ����ʾ��
    BuzzerWarnning,                     //������ʾ��
    BuzzerSysStart,                     //������ʾ��
    BuzzerDing,                         //������һ��
    BuzzerFinish,                       //������ʾ��
}BuzzerEnum;


//������������غ���
void Buzzer_Init        (void);
void Buzzer_Timer       (void);
void Buzzer_Handle      (void);
void Buzzer_Enable      (BuzzerEnum buzzer);


//�����ṹ��
extern BuzzerStruct buzzerStr;


#endif /* CODE_BUZZER_BUZZER_H_ */
