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
    I2C 无需配置 直接用驱动 占PB6, PB7
*/

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

#pragma region 串口空闲中断 可接受不定长数据
// 数据定长则直接用接收完成回调函数
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

#pragma region 延时执行函数的方法
//设置任意定时器 开启溢出中断 在中断回调函数中调用tickDelays() (或直接把tickDelays改成HAL_TIM_PeriodElapsedCallback函数)
//调用newDelay可以自动配置新的延时任务
//参数active:   是否计时 false则不进行任何操作 直到被改为true
//参数invoke:   延时结束后调用的函数
//参数delay:    调用invoke前的溢出次数 note:无法避免1次溢出时间以内的偏差 在执行频率*不影响运行*的条件下 最好把频率设置较高
//参数activate: 指向另一个DelayedAction类型的指针 在指向的任务延时结束 invoke调用完毕后 将当前任务active自动设置为true 在下一次溢出开始计时 不需要这个功能可传入NULL

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
