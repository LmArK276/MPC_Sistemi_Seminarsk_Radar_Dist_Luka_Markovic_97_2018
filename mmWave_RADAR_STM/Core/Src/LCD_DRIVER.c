#include "main.h"

//pin declarations
#define RS GPIO_PIN_12
#define EN GPIO_PIN_13

void lcdStrobe()
{
	HAL_GPIO_WritePin(GPIOA, EN, 1);
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOA, EN, 0);
}

void lcdWriteData(uint8_t data)
{
	GPIOA->ODR = data;
	HAL_GPIO_WritePin(GPIOA, RS, 1); //set RS to HIGH to send data
	lcdStrobe();
}

void lcdWriteCmd(uint8_t data)
{
	GPIOA->ODR = data;
	HAL_GPIO_WritePin(GPIOA, RS, 0);//set RS to LOW to send commands
	lcdStrobe();
}

void lcdInit()
{
	HAL_Delay(50);
	lcdWriteCmd(0x38);
	lcdStrobe();

	lcdWriteCmd(0x0C);
	lcdStrobe();

	lcdWriteCmd(0x06);
	lcdStrobe();

	lcdWriteCmd(0x01);
	lcdStrobe();

	lcdWriteCmd(0x80);
	lcdStrobe();
}


void lcdWriteString(char const *s)
{
	while(*s)
	{
		lcdWriteData(*s++);
	}

}

void lcdClear()
{
	lcdWriteCmd(0x01);
}



