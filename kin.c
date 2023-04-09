#include <stdint.h>
#include <stdbool.h>

uint64_t b1 = 0;
uint64_t b2 = 0; // cap: 0.64s
bool b1a = true;
bool b2a = true;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    { // 100Hz
        b1 = b1 << 1 | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
        b2 = b2 << 1 | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

        if (b1a && !(b1 & 0x00000000000000FF))
        { // short click implement
            b1a = false;
            printf("B1 click\n");
        }
        else
            b1a = b1 & 0x00000000000000FF;

        if (!(b2 & 0x0FFFFFFFFFFFFFF) || (b2a && !(b2 & 0x00000000000000FF)))
        { // long/short click
            b2a = false;
            printf("B2 triggered\n");
        }
        else
            b2a = b2 & 0x000000000000FF;
        // printf("0x%16llX\n", b1);
    }
}

void eeWrite8(uint8_t addr, uint8_t val)
{
    I2CStart();
    I2CSendByte(0xA0);
    I2CWaitAck();
    I2CSendByte(addr);
    I2CWaitAck();
    I2CSendByte(val);
    I2CWaitAck();
    I2CStop();
}

uint8_t eeRead8(uint8_t addr)
{
    I2CStart();
    I2CSendByte(0xA0);
    I2CWaitAck();
    I2CSendByte(addr);
    I2CWaitAck();
    I2CStop();
    I2CStart();
    I2CSendByte(0xA1);
    I2CWaitAck();
    uint8_t r = I2CReceiveByte();
    I2CSendNotAck();
    I2CStop();
    return r;
}

void eeWrite16(uint8_t addr, uint16_t val)
{
    I2CStart();
    I2CSendByte(0xA0);
    I2CWaitAck();
    I2CSendByte(addr);
    I2CWaitAck();
    I2CSendByte(val >> 8);
    I2CWaitAck();
    I2CSendByte(val & 0x00FF);
    I2CWaitAck();
    I2CStop();
}

void eeWriteAny(uint8_t addr, void *data, uint16_t size)
{
    uint8_t *p = data;
    register int s = size;
    eeWrite8(addr, *p);
    while (--s)
    {
        HAL_Delay(5);
        eeWrite8(++addr, *(++p));
    }
}

void *eeReadAny(uint8_t addr, uint16_t size)
{
    void *r = malloc(size);
    uint8_t *p = r;
    *p = eeRead8(addr);
    while (--size)
    {
        HAL_Delay(5);
        *(++p) = eeRead8(++addr);
    }
    return r;
}

void eeDump()
{
    // uint8_t val;
    HAL_UART_Transmit(&huart1, (uint8_t *)"0x_-  0 1 2 3 4 5 6 7 8 9 A B C D E F", 37, 0xFFFF);
    uint8_t vl = 0x00;
    for (register int addr = 0x00; addr < 0x100; addr++)
    {
        if (addr == vl)
        {
            printf("\n0x%1x- ", addr >> 4);
            vl += 0x10;
        }
        printf("%x", eeRead8(addr));
        HAL_Delay(5);
    }
}

