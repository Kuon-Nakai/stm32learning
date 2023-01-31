#include <stdint.h>
/* ���ű�������
��������
    ���� B1 - PB0, B2 - PB1, B3 - PB2, B4 - PA0
    LED  LD1 - PC8, LD2 - PC9, LD3 - PC10, LD4 - PC11, LD5 - PC12, LD6 - PC13, LD7 - PC14, LD8 - PC15
        ����PD2����������
        ��HSIʱ��Դ
    ��ѹ�ɼ� 1 -(J11)- PB15 , 2 -(J12)- PB2
    Ƶ����� 1 -(J9)- PB4   , 2 -(J10)- PA15
    I2C �������� ֱ�������� ռPB6, PB7
*/

//���ô���Ƭ��

//include
#include <stdio.h> //sprintf() / vsprintf()
#include <stdarg.h>//vararg
#include <stdbool.h>
#include <string.h>//memset()
#include <stdlib.h>//malloc(), free()
//#include "lcd.h"
//#include "i2c.h"



//LED���ٲ���
#define ld(x, s)\
    HAL_GPIO_WritePin(GPIOC, 0x40 << x, s); \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);\
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)



//��ȡ����״̬
#define isKeyUp(x) HAL_HPIO_ReadPin(x == 4 ? GPIOA : GPIOB, (x - 1) % 3)



//�ַ����������
uint8_t *format(char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);

    //����д�� ��_vscprintf()��keil�����޷�ʹ��:
    uint8_t *buffer = (uint8_t *)malloc(_vscprintf(fmt, ap) + 1);
    //��̬����
    uint8_t *buffer = (uint8_t *)malloc(32);
    //��̬����
    memset(buffer, 0, 32);
    
    vsprintf(buffer, fmt, ap);
    return buffer;
    //��̬�����ڴ�ʹ�ú�free()
}



//����������
//method 1: ����ɨ��
void main_(void) {
    //...
    while(1) {
        if(isKeyUp(1)){}
        if(isKeyUp(2)){}
        //...
    }
}
//method 2:EXTI�ж�
//����B1 B2 B3(PB0 PB1 PB2)ΪEXTI 0 1 2�� ����B4ΪCOMP3������ VrefΪ������ ȫ�������½��ز��� ����EXTI0 1 2�ߺ�COMP3�ж� ���ȼ�3
//�жϻص�����:
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) { //b4
    //�߼������ж�
    HAL_Delay(32);
    if(isKeyUp(4)) return;
    //������Ӧ����
}
void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    switch(pin) {
        case GPIO_PIN_0: //b1
            //...
            break;
        case GPIO_PIN_1: //b2
            //...
            break;
        case GPIO_PIN_2: //b3
            //...
            break;
    }
}

