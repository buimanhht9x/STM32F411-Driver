/**
  ******************************************************************************
  * @file    main.c
  * @author  Auto-generated by STM32CubeIDE
  * @version V1.0
  * @brief   Default main function.
  ******************************************************************************
*/

#include <stdio.h>

/* global variables */
int g_data1 = -4000;
int g_data2 = 200;
int result = 0;

int main(void)
{
	result = g_data1 + g_data2;

	printf("Result = %d\n",result);

	for(;;);
}