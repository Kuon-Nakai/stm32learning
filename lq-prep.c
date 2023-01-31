#include <stdint.h>
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



//LED快速操作
#define ld(x, s)\
    HAL_GPIO_WritePin(GPIOC, 0x40 << x, s); \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);\
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET)



//获取按键状态
#define isKeyUp(x) HAL_HPIO_ReadPin(x == 4 ? GPIOA : GPIOB, (x - 1) % 3)



//字符串输出处理
uint8_t *format(char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);

    //最优写法 但_vscprintf()在keil可能无法使用:
    uint8_t *buffer = (uint8_t *)malloc(_vscprintf(fmt, ap) + 1);
    //动态分配
    uint8_t *buffer = (uint8_t *)malloc(32);
    //静态缓冲
    memset(buffer, 0, 32);
    
    vsprintf(buffer, fmt, ap);
    return buffer;
    //动态分配内存使用后free()
}



//按键输入检测
//method 1: 按键扫描
void main_(void) {
    //...
    while(1) {
        if(isKeyUp(1)){}
        if(isKeyUp(2)){}
        //...
    }
}
//method 2:EXTI中断
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

