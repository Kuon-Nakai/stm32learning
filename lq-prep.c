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
    I2C PB6, PB7
*/

//����startup����BKPT 0xAB, ��Options for target / target�п���Use MicroLIB
//memcpy() sizeָ�ڴ渴���ֽ��� ������Ԫ������
//I2C����������ʼ������ ��Ҫ���úö�Ӧ����
//��������û�к���TIM���� ��������ģ��Ƚ���remap�����ʵ�TIM

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
//TIM��ΪReset mode, TI1FP_.
//����������ΪInput capture direct mode
//�յ������źź���ȴ����ж� ���Զ�����������λ ���Լ򻯴���
//ע���ȡ����ֵ��HAL_TIM_ReadCapturedValue() ������GetCounter�� ��Ϊ������ֵ������仯
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIMx) {//�ж�
        int freq = clk / psc / ckd / HAL_TIM_ReadCapturedValue(&htimx, TIM_CHANNEL_x);
        //...
    }
}
#pragma endregion

#pragma region PWM����
//Alt: Combined channel - PWM input
//ԭ������ͬ

//TIM��ΪReset mode, ����Դ��Ϊ��������
//����������ΪInput capture direct mode, �����ز���
//����һ������ΪInput capture indirect mode, �½��ز���
//direct���Ų���ֵ��Ϊ���� indirect���Ų���ֵΪ�ߵ�ƽʱ��
//ע���ж�����ʱ��Ҫ��HAL_TIM_ACTIVE_CHANNEL_x ������TIM_CHANNEL_x
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIMx && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_x) {
        int t = HAL_TIM_ReadCapturedValue(&htimx, TIM_CHANNEL_x);
        float d = HAL_TIM_ReadCapturedValue(&htimx, TIM_CHANNEL_y) / HAL_TIM_ReadCapturedValue(&htimx, TIM_CHANNEL_x);
    }
}
#pragma endregion

#pragma region ���ڿ����ж�
//��UART��չ�� ��������С
void main_() {
    //...init
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)rxBuf, (rxSize));
}
//�ص�����
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    if(huart->Instance == USART1) {
        //...
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)rxBuf, (rxSize)); //���´�
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

#pragma region EEPROM��д����
//ע����CubeMX����PB6 PB7��GPIO Output.
//ע����ÿ�ζ�д��ȷ����ʱ5ms
//0xA0 д/��λ, 0xA1��
//������ָ��ʵ�ֶ��������͵Ķ�д

/**
 * @brief       Writes value of 1 byte to the specified address in EEPROM.
 * @param addr  Target address
 * @param data  Value to be written
 * @note        Supported types: bool, (un)signed char, (u)int8_t, (u)int_fast8_t, (u)int_least8_t
 */
void eeWrite8(uint8_t addr, uint8_t data)
{
    I2CStart();
    I2CSendByte(0xA0);
    I2CWaitAck();
    I2CSendByte(addr);
    I2CWaitAck();
    I2CSendByte(data);
    I2CWaitAck();
    I2CStop();
}

uint8_t eeRead8(uint8_t addr)
{
    uint8_t eerBuf;
    I2CStart();
    I2CSendByte(0xA0);
    I2CWaitAck();
    I2CSendByte(addr);
    I2CWaitAck();
    I2CStop();
    I2CStart();
    I2CSendByte(0xA1);
    I2CWaitAck();
    eerBuf = I2CReceiveByte();
    I2CSendNotAck();
    I2CStop();
    return eerBuf;
}

/**
 * @brief       Writes value of 2 bytes to the specified address in EEPROM.
 * @param addr  Target address
 * @param data  Value to be written
 * @note        Supported types: (un)signed short, (u)int16_t, (u)int_least16t
 */
void eeWrite16(uint8_t addr, uint16_t data)
{
    eeWrite8(addr, data >> 8);
    eeWrite8(addr + 1, data & 0x00FF);
}

uint16_t eeRead16(uint8_t addr)
{
    return (eeRead8(addr) << 8) | eeRead8(addr + 1);
}

/**
 * @brief       Writes value of ANYTHING to the specified address in EEPROM.
 * @param addr  Target address
 * @param data  Pointer to value
 * @param size  Length of the value, in bytes
 * @note        Supported types: Anything. Structs even.
 * @warning     DON'T YOU DARE MAKE IT OVERFLOW!
 */
void eeWriteAny(uint8_t addr, void *data, uint8_t size)
{
    uint8_t *dat = data;
    while (size--)
            eeWrite8(addr++, *dat++);
}

/**
 * @brief       Reads value of ANYTHING to the specified address in EEPROM.
 * @param addr  Target address
 * @param size  Length of the value, in bytes
 * @retval      Pointer to requested data. Type is unspecified and needs to be casted.
 * @note        Supported types: Anything. Structs even.
 * @warning     Use free() to release the buffer after use!
 */
void *eeReadAny(uint8_t addr, uint8_t size)
{
    void *r = malloc(size);
    if (r != 0)
    {
            uint8_t *p = r;
            while (size--)
                *p++ = eeRead8(addr++);
    }
    return r;
}
#pragma endregion

#pragma region ��ʱִ�к����ķ���
//�������ⶨʱ�� ��������ж� ���жϻص������е���tickDelays() (��ֱ�Ӱ�tickDelays�ĳ�HAL_TIM_PeriodElapsedCallback����)
//����newDelay�����Զ������µ���ʱ����
//����active:   �Ƿ��ʱ false�򲻽����κβ��� ֱ������Ϊtrue
//����invoke:   ��ʱ��������õĺ���
//����delay:    ����invokeǰ��������� note:�޷�����1�����ʱ�����ڵ�ƫ�� ��ִ��Ƶ��*��Ӱ������*�������� ��ð�Ƶ�����ýϸ�
//����activate: ָ����һ��DelayedAction���͵�ָ�� ��ָ���������ʱ���� invoke������Ϻ� ����ǰ����active�Զ�����Ϊtrue ����һ�������ʼ��ʱ ����Ҫ������ܿɴ���NULL

typedef struct {
    bool active;            //Whether the delay should be ticked
    uint32_t delay;         //Reduced by 1 every tick
    void (*invoke)(void);   //Function pointer, to be called when delay reaches 0
    void *next;             //Next element in chain. Nullable
    void *prev;             //Prev element in chain. Nullable
    void *activate;         //Activates the specified delayed action after this action is performed. Nullable
} DelayedAction;

DelayedAction *first = NULL;
DelayedAction *last  = NULL;

DelayedAction *newDelay(bool active, uint32_t delay, void (*invoke)(void), DelayedAction *activateBy) {
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

#pragma region ��ʱ�����Ľ� - ������
//�������ö�ʱ��������Դռ�ò��򻯴洢�ṹ

typedef struct _DelayedFunction {
    //DelayedFunction *prev;
    DelayedFunction *next;
    DelayedFunction *trig;
    void (*invoke)(void);
    uint32_t delay;
} DelayedFunction;

DelayedFunction *currentDelay;
TIM_HandleTypeDef *hdelaytim;
uint32_t delayCh;

DelayedFunction *addDelay(uint32_t t, void (*func)(void), DelayedFunction *after) {
    DelayedFunction *r;
    r->invoke = func;
    r->trig = after;
    if(after) {
        if(after->next) {
            if(after->next->next) r->next = after->next->next;
            after->next = r;
            r->delay = t;
            return r;
        }
        after->next = r;
        //TODO:
    }
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

#pragma region ���нṹ
typedef struct _QueueItem {
    void *val;
    QueueItem *next;
    QueueItem *prev;
} QueueItem;
typedef struct _Queue {
    int capacity;
    QueueItem *first;
    QueueItem *last;
    void (*onForcedOut)(void *);
} Queue;
inline void qpush(Queue *q, void *value) {
    QueueItem *qi = (QueueItem *)malloc(sizeof(QueueItem));
    qi->val = value;
    if(q->capacity) {
        q->capacity--;
        if(q->first) {
            q->last->next = qi;
            q->last = qi;
            return;
        }
        q->first = qi;
        q->last = qi;
        return;
    }
    QueueItem *p = q->last;
    q->last = q->last->prev;
    q->onForcedOut(p->val);
    free(p);
    p = NULL;
    qi->next = q->first;
    q->first = qi;
}
inline void *qpull(Queue *q) {
    void *v = q->last->val, *p = q->last;
    q->capacity++;
    q->last = q->last->prev;
    free(p);
    p = NULL;
    return v;
}
inline void qrm(Queue *q) {
    QueueItem *p = q->first, *p1 = p;
    while(p) {
        p = p1->next;
        free(p1);
        p1 = p;
    }
    free(q);
}
inline Queue *qnew(int len, void (*onForcedOut)(void *)) {
    Queue *q = (Queue *)malloc(sizeof(Queue));
    q->capacity = len;
    q->onForcedOut = onForcedOut;
    q->first = NULL;
    q->last = NULL;
    return q;
}
#pragma endregion

#define getKeyRange(v)                                                                                                           \
    ((v < 2500) ? (v < 1500) ? 3 - !(v & 0xFF00) - !(v & 0xF300) : 5 - !(v & 0xF700) : (v < 3600) ? 6 + (v & 0x0400) \
                                                                                                                : (~v & 0x0F80) << 3)



uint8_t getKeyRange_forHuman(uint16_t v) {
    if (v < 2500) {
        if (v < 1500) {
            if (v < 256) return 1;
            else if (v < 1024) return 2;
            else return 3;
        }
        else if(v < 2048) return 4;
        else return 5;
    }
    else {
        if(v < 3600) {
            if(v < 3000) return 6;
            else return 7;
        }
        else {
            if(v < 4000) return 8;
            else return 0;
        }
    }
}

void main() {
    
}
