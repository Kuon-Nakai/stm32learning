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
/* 蓝桥杯竞赛板
常用引脚
    按键 B1 - PB0, B2 - PB1, B3 - PB2, B4 - PA0
    LED  LD1 - PC8, LD2 - PC9, LD3 - PC10, LD4 - PC11, LD5 - PC12, LD6 - PC13, LD7 - PC14, LD8 - PC15
        需用PD2控制锁存器
        用HSI时钟源
    电压采集 1 -(J11)- PB15 , 2 -(J12)- PB2
    频率输出 1 -(J9)- PB4   , 2 -(J10)- PA15
    I2C PB6, PB7
*/

//遇到startup卡在BKPT 0xAB, 在Options for target / target中开启Use MicroLIB
//memcpy() size指内存复制字节数 并不是元素数量
//I2C驱动不含初始化代码 需要设置好对应引脚
//输入引脚没有合适TIM可用 可以利用模拟比较器remap到合适的TIM

//常用代码片段

//include
#include <stdio.h> //sprintf() / vsprintf()
#include <stdarg.h>//vararg
#include <stdbool.h>
#include <string.h>//memset()
#include <stdlib.h>//malloc(), free()
//#include "lcd.h"
//#include "i2c.h"

#pragma region LED快速操作
#define lds(x, s)\
    HAL_GPIO_WritePin(GPIOC, 0x80 << x, s); \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);\
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)
#define ldt(x)\
    HAL_GPIO_TogglePin(GPIOC, 0x80 << x); \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);\
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)
#pragma endregion

#pragma region 获取按键状态
#define isKeyUp(x) HAL_HPIO_ReadPin(x == 4 ? GPIOA : GPIOB, (x - 1) % 3)
#pragma endregion

#pragma region 字符串格式化输出处理
uint8_t *format(char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    //最优写法 但 _vscprintf() 在keil可能无法使用:
    uint8_t *buffer = (uint8_t *)malloc(_vscprintf(fmt, ap) + 1);
    //定长 动态分配
    uint8_t *buffer = (uint8_t *)malloc(32);
    //定长 静态缓冲
    memset(buffer, 0, 32);
    
    vsprintf(buffer, fmt, ap);
    return buffer;
    //动态分配内存使用后free()
}
#pragma endregion

#pragma region ADC校准, 开启(DMA)
void main_() {
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, /*变量或数组指针*/x, /*数组长度*/x);
}
#pragma endregion

#pragma region 读取频率
//TIM设为Reset mode, TI1FP_.
//输入引脚设为Input capture direct mode
//收到输入信号后会先触发中断 再自动将计数器复位 可以简化代码
//注意读取捕获值用HAL_TIM_ReadCapturedValue() 而不用GetCounter宏 因为计数器值会继续变化
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIMx) {//判断
        int freq = clk / psc / ckd / HAL_TIM_ReadCapturedValue(&htimx, TIM_CHANNEL_x);
        //...
    }
}
#pragma endregion

#pragma region PWM输入
//Alt: Combined channel - PWM input
//原理上相同

//TIM设为Reset mode, 触发源设为输入引脚
//输入引脚设为Input capture direct mode, 上升沿捕获
//另设一个引脚为Input capture indirect mode, 下降沿捕获
//direct引脚捕获值即为周期 indirect引脚捕获值为高电平时间
//注意判断引脚时需要用HAL_TIM_ACTIVE_CHANNEL_x 而不是TIM_CHANNEL_x
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIMx && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_x) {
        int t = HAL_TIM_ReadCapturedValue(&htimx, TIM_CHANNEL_x);
        float d = HAL_TIM_ReadCapturedValue(&htimx, TIM_CHANNEL_y) / HAL_TIM_ReadCapturedValue(&htimx, TIM_CHANNEL_x);
    }
}
#pragma endregion

#pragma region 串口空闲中断
//用UART拓展库 工作量较小
void main_() {
    //...init
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)rxBuf, (rxSize));
}
//回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    if(huart->Instance == USART1) {
        //...
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)rxBuf, (rxSize)); //重新打开
    }
}
#pragma endregion

//按键输入检测
#pragma region method 1: 按键扫描
void main_(void) {
    //...
    while(1) {
        if(isKeyUp(1)){}
        if(isKeyUp(2)){}
        //...
    }
}
#pragma endregion
#pragma region method 2: GPIO+COMP EXTI中断
//配置B1 B2 B3(PB0 PB1 PB2)为EXTI 0 1 2线 配置B4为COMP3正输入 Vref为负输入 全部设置下降沿捕获 开启EXTI0 1 2线和COMP3中断 优先级3
//中断回调函数:
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) { //b4
    //逻辑条件判断
    HAL_Delay(32);
    if(isKeyUp(4)) return;
    //按键响应操作
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

#pragma region EEPROM读写函数
//注意在CubeMX开启PB6 PB7的GPIO Output.
//注意在每次读写后确保延时5ms
//0xA0 写/定位, 0xA1读
//可利用指针实现对任意类型的读写

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

#pragma region 延时执行函数的方法
//设置任意定时器 开启溢出中断 在中断回调函数中调用tickDelays() (或直接把tickDelays改成HAL_TIM_PeriodElapsedCallback函数)
//调用newDelay可以自动配置新的延时任务
//参数active:   是否计时 false则不进行任何操作 直到被改为true
//参数invoke:   延时结束后调用的函数
//参数delay:    调用invoke前的溢出次数 note:无法避免1次溢出时间以内的偏差 在执行频率*不影响运行*的条件下 最好把频率设置较高
//参数activate: 指向另一个DelayedAction类型的指针 在指向的任务延时结束 invoke调用完毕后 将当前任务active自动设置为true 在下一次溢出开始计时 不需要这个功能可传入NULL

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

#pragma region 延时函数改进 - 待测试
//尝试利用定时器降低资源占用并简化存储结构

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

//拓展板模块
#pragma region 数码管
//连接 SER RCK SCK 控制锁存器
//SER - 数据线, RCK - 上升沿输出存储值, SCK - 上升沿存入SER的状态并将存储左移一位
//注意函数需要从n3到n1操作
//GPIO PA1 PA2 PA3 
uint8_t Seg[10] = {0b1110'1110, 0b0010'1000, 0b1100'1101, 0b0110'1101, 0b0010'1011, 0b0110'0111, 0b1110'0111, 0b0010'1100, 0b1110'1111, 0b01101111};
uint8_t SegDot  = 0b0001'0000;
//三个参数: Seg[数字]可显示相应数字 或 (Seg[数字] | SegDot) 显示加点的数字
void SegShow(uint8_t n1, uint8_t n2, uint8_t n3) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // RCK
    for(int i = 8; i--; ) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, !(n3 & 0b1000'0000)); // 判断最高位 输出到SER
        n3 <<= 1;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); // SCK生成上升沿 写入存储
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
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // RCK上升沿 输出到数码管
}
#pragma endregion

#pragma region ADC按键
//按键识别可用ADC读取电压, 判断范围来实现
//引脚: PA5 - ADC2_IN13
void main_() {
    HAL_ADCEx_Calibration_Start(&hadc2);
    HAL_ADC_Start_IT(&hadc2);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if(hadc->Instance != ADC2) return;
    HAL_ADC_GetValue(&hadc2);
    //判断 执行
}
#pragma endregion

#pragma region 队列结构
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
