/***********************************************************************************************************************
* File Name    : KeyboardHoldRepeat.c
* Version      : 
* Device(s)    : Keyboard Handler code
* Tool-Chain   : IAR Systems ARM
* Description  : This file implements support code for a keyCode that is a result of a keyboard or button scan
*              : 
* Creation Date: 10OCT2021
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "main.h"
#include "KeyboardHoldRepeat.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>



/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/


/***********************************************************************************************************************
Global/module variables
***********************************************************************************************************************/
extern uint8_t processKeyCode;
extern uint8_t keyCodeProcessed;

extern UART_HandleTypeDef huart1;

volatile uint8_t kbdTest = 0;
char testChar[1] = {'!'};


/***********************************************************************************************************************
module function prototypes
***********************************************************************************************************************/



/***********************************************************************************************************************
code start
***********************************************************************************************************************/
void ProcessKeyCode(uint8_t _kcode)
{
  //kbdTest = 27;
  
  switch (_kcode) {
  case 0x06:  // single keys:   1
    //kbdTest = 1;
    //kbdTest++;
    HAL_UART_Transmit(&huart1,testChar, 1, 1000);
    if (++testChar[0] > '}') testChar[0] = '!';
    break;
  case 0x05:  //                2
    kbdTest = 2;
    break;
  case 0x03:  //                3
    kbdTest = 37;
    break;
  case 0x02:  // 2-key chords:  1+3
    kbdTest = 4;
    break;
  default:
    break;
  }
  
  processKeyCode = false;
  keyCodeProcessed = true;
  
}



uint8_t ValidKeyCode(uint8_t _kcode)
{
  uint8_t validKeyCode = false;
  
  switch (_kcode) {
  case 0x06:  // single keys:   1
  case 0x05:  //                2
  case 0x03:  //                3
  case 0x02:  // 2-key chords:  1+3
    validKeyCode = true;
    break;
  default:
    break;
  }
  return validKeyCode;
}


uint8_t ScanKeyboard(void)
{
  uint8_t keyCode = NO_KEY_PRESSED;
  
  keyCode = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) << 2) | (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) << 1) | (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) << 0);
  
  return keyCode;
}
