#include "ds18b20_ll.h"

#define delay_us(X)  delay((X)*80/5)

void delay(unsigned int n)
{
    while(n--);
}

void ds18b20_init_x(void)
{
    LL_GPIO_InitTypeDef	GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    LL_APB2_GRP1_EnableClock(OW_PIN_CLOCK);


    GPIO_InitStruct.Pin = OW_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH ;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP ;
    LL_GPIO_Init(OW_PIN_PORT, &GPIO_InitStruct);
}
//
void mode_input1(void )
{

    LL_GPIO_InitTypeDef	GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */

    GPIO_InitStruct.Pin = OW_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(OW_PIN_PORT, &GPIO_InitStruct);
}

void mode_output1(void )
{

    LL_GPIO_InitTypeDef	GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = OW_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(OW_PIN_PORT, &GPIO_InitStruct);
}

//
uint8_t ow_reset(void)
{
    uint8_t err;

    OW_DIR_OUT(); // pull OW-Pin low for 480us
    OW_OUT_LOW(); // disable internal pull-up (maybe on from parasite)

    delay_us(400);	  //about 480us

    // set Pin as input - wait for clients to pull low
    OW_DIR_IN(); // input

    delay_us(66);
    err = OW_GET_IN();		// no presence detect
    // nobody pulled to low, still high


    // after a delay the clients should release the line
    // and input-pin gets back to high due to pull-up-resistor
    delay_us(480 - 66);
    if( OW_GET_IN() == 0 )		// short circuit
        err = 1;

    return err;
}

uint8_t ow_bit_io( uint8_t b )
{
    OW_DIR_OUT(); // drive bus low
    OW_OUT_LOW();
    delay_us(1); // Recovery-Time wuffwuff was 1

    if ( b ) OW_DIR_IN(); // if bit is 1 set bus high (by ext. pull-up)

#define  OW_CONF_DELAYOFFSET  5
    delay_us(15 - 1 - OW_CONF_DELAYOFFSET);

    if( OW_GET_IN() == 0 ) b = 0;  // sample at end of read-timeslot

    delay_us(60 - 15);
    OW_DIR_IN();

    return b;
}

uint8_t ow_byte_wr( uint8_t b )
{
    uint8_t i = 8, j;
    do
    {
        j = ow_bit_io( b & 1 );
        b >>= 1;
        if( j ) b |= 0x80;
    }
    while( --i );
    return b;
}

//
uint8_t ow_byte_rd( void )
{
    return ow_byte_wr( 0xFF );
}
