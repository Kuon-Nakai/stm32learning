//stuff for intellisense, ignore these.
#include <stdint.h>
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
int *x;
typedef void *COMP_HandleTypeDef;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_usart1_rx;
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

#pragma region LED���ٲ���
#define lds(x, s)\
    HAL_GPIO_WritePin(GPIOC, 0x80 << x, s); \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);\
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)
#define ldt(x)\
    HAL_GPIO_TogglePin(GPIOC, 0x80 << x); \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);\
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)
#pragma endregion

#pragma region ��ȡ����״̬
#define isKeyUp(x) HAL_HPIO_ReadPin(x == 4 ? GPIOA : GPIOB, (x - 1) % 3)
#pragma endregion

#pragma region �ַ�����ʽ���������
uint8_t *format(char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    //����д�� �� _vscprintf() ��keil�����޷�ʹ��:
    uint8_t *buffer = (uint8_t *)malloc(_vscprintf(fmt, ap) + 1);
    //���� ��̬����
    uint8_t *buffer = (uint8_t *)malloc(32);
    //���� ��̬����
    memset(buffer, 0, 32);
    
    vsprintf(buffer, fmt, ap);
    return buffer;
    //��̬�����ڴ�ʹ�ú�free()
}
#pragma endregion

#pragma region ADCУ׼, ����(DMA)
void main_() {
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, /*����������ָ��*/x, /*���鳤��*/x);
}
#pragma endregion

#pragma region ��ȡƵ��
uint32_t freq = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM2) {
        uint32_t val = __HAL_TIM_GetCounter(&htim2);
        __HAL_TIM_SetCounter(&htim2, 0);
        HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
        freq = /*some number*/ 1000000 / val;
    }
}
#pragma endregion

#pragma region ���ڿ����ж� �ɽ��ܲ���������
// ���ݶ�����ֱ���ý�����ɻص�����
uint8_t rxBuf[32] = {0};

void main_() {
    //Init...
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, rxBuf, 32);
}
//stm32g4xx_it.c
void USART1_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UART_DMAStop(&huart1);
        int len = 32 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        
    }
}
#pragma endregion

//����������
#pragma region method 1: ����ɨ��
void main_(void) {
    //...
    while(1) {
        if(isKeyUp(1)){}
        if(isKeyUp(2)){}
        //...
    }
}
#pragma endregion
#pragma region method 2: GPIO+COMP EXTI�ж�
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
#pragma endregion

#pragma region ��ʱִ�к����ķ���
//�������ⶨʱ�� ��������ж� ���жϻص������е���tickDelays() (��ֱ�Ӱ�tickDelays�ĳ�HAL_TIM_PeriodElapsedCallback����)
//����newDelay�����Զ������µ���ʱ����
//����active:   �Ƿ��ʱ false�򲻽����κβ��� ֱ������Ϊtrue
//����invoke:   ��ʱ��������õĺ���
//����delay:    ����invokeǰ��������� note:�޷�����1�����ʱ�����ڵ�ƫ�� ��ִ��Ƶ��*��Ӱ������*�������� ��ð�Ƶ�����ýϸ�
//����activate: ָ����һ��DelayedAction���͵�ָ�� ��ָ���������ʱ���� invoke������Ϻ� ����ǰ����active�Զ�����Ϊtrue ����һ�������ʼ��ʱ ����Ҫ������ܿɴ���NULL

typedef void *Function();
typedef struct {
    bool active;        //Whether the delay should be ticked
    uint32_t delay;      //Reduced by 1 every tick
    Function *invoke;   //Function pointer, to be called when delay reaches 0
    void *next;         //Next element in chain. Nullable
    void *prev;         //Prev element in chain. Nullable
    void *activate;     //Activates the specified delayed action after this action is performed. Nullable
} DelayedAction;

DelayedAction *first = NULL;
DelayedAction *last  = NULL;

DelayedAction *newDelay(bool active, uint32_t delay, Function *invoke, DelayedAction *activateBy) {
    DelayedAction *act = (DelayedAction *)malloc(sizeof(DelayedAction));
    if(act == NULL) return NULL;
    act->active = active;
    act->invoke = invoke;
    act->delay  = delay;
    act->next   = NULL;
    act->prev   = last;
    activateBy->activate = act;
    if(last) {
        last->next  = act;
        last        = act;
    }
    if(first == NULL) {
        first = act;
        last = act;
    }
    return act;
}

void tickDelays() {
    DelayedAction *act = first;
    DelayedAction *temp = NULL;
    if(act == NULL) return;
    if(act->active) {
        if ((act->delay)-- == 0) {
            act->invoke();
            if(act->activate) ((DelayedAction *)act->activate)->active = true;
            first = (DelayedAction *)act->next;
            if(act->next) ((DelayedAction *)act->next)->prev = NULL;
            temp = (DelayedAction *)act->next;
            free(act);
        }
        else temp = (DelayedAction *)act->next;
    }
    else temp = (DelayedAction *)act->next;
    
    if(temp == NULL) return;
    do {
        act = temp;
        if(act->active) {
            if ((act->delay)-- == 0) {
                act->invoke();
                if (act->activate) ((DelayedAction *)act->activate)->active = true;
                if (act->prev) ((DelayedAction *)act->prev)->next = (DelayedAction *)act->next;
                if (act->next) ((DelayedAction *)act->next)->prev = (DelayedAction *)act->prev;
                temp = (DelayedAction *)act->next;
                free(act);
            }
            else temp = (DelayedAction *)act->next;
        }
        else temp = (DelayedAction *)act->next;
    }while(temp != NULL);
}
#pragma endregion

//��չ��ģ��
#pragma region �����
//���� SER RCK SCK ����������
//SER - ������, RCK - ����������洢ֵ, SCK - �����ش���SER��״̬�����洢����һλ
//ע�⺯����Ҫ��n3��n1����
//GPIO PA1 PA2 PA3 
uint8_t Seg[10] = {0b1110'1110, 0b0010'1000, 0b1100'1101, 0b0110'1101, 0b0010'1011, 0b0110'0111, 0b1110'0111, 0b0010'1100, 0b1110'1111, 0b01101111};
uint8_t SegDot  = 0b0001'0000;
//��������: Seg[����]����ʾ��Ӧ���� �� (Seg[����] | SegDot) ��ʾ�ӵ������
void SegShow(uint8_t n1, uint8_t n2, uint8_t n3) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // RCK
    for(int i = 8; i--; ) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, !(n3 & 0b1000'0000)); // �ж����λ �����SER
        n3 <<= 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); // SCK���������� д��洢
    }
    for (int i = 8; i--;) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, !(n2 & 0b1000'0000));
        n2 <<= 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    }
    for (int i = 8; i--;) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, !(n1 & 0b1000'0000));
        n1 <<= 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // RCK������ ����������
}
#pragma endregion


#pragma region ADC����
//����ʶ�����ADC��ȡ��ѹ, �жϷ�Χ��ʵ��
//����: PA5 - ADC2_IN13
void main_() {
    HAL_ADCEx_Calibration_Start(&hadc2);
    HAL_ADC_Start_IT(&hadc2);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if(hadc->Instance != ADC2) return;
    HAL_ADC_GetValue(&hadc2);
    //�ж� ִ��
}
#pragma endregion
