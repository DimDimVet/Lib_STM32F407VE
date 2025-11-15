#ifndef APP_H_
#define APP_H_

/*-----------------------------------------------------------------------------------------------*/
#include "lib_stm32.h"
#include "GPIO_STM32F407.h"
#include "RCC_STM32F407.h"
#include "USB_FS_STM32F407.h"

#include <string.h>

void Work_USB();
void Work_MSO();
void Work_USART();
/*-----------------------------------------------------------------------------------------------*/

#endif