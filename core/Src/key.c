#include "key.h"
#include "main.h"

extern uint16_t SET_value;
int key_read()
{
	if(HAL_GPIO_ReadPin( KEY_1_GPIO_Port, KEY_1_Pin)==0)
	{
		HAL_Delay(50);
	return 1;
	}
	else if(HAL_GPIO_ReadPin( KEY_2_GPIO_Port, KEY_2_Pin)==0)
	{
		HAL_Delay(50);
	return 2;
	}

	else if(HAL_GPIO_ReadPin( KEY_3_GPIO_Port, KEY_3_Pin)==0)
	{
		HAL_Delay(50);
	return 3;		
	}
	else return 0;
}


void key()
{
if (key_read()==1 & SET_value<1451)
	{
	SET_value=SET_value+50;
	}
else if (key_read()==2)
	{
	SET_value=0;
	}
else if (key_read()==3 & SET_value>549)
	{
	SET_value=SET_value-50;
	}
	else SET_value=SET_value;
}