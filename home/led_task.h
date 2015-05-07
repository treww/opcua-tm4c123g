

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

// Color of led
typedef enum
{
  Red  = 0x02,
  Blue = 0x04,
  Green = 0x08
} LedColor;

typedef struct
{
  int8_t Color;
  int8_t IsOn;
} LedAction;

// Enable/Disable led with specified color
void EnableLed(const LedAction* action);

// Enable/Disable led with specified color
/*
// this method should be called fro ISR.
void EnableLedFromISR(LedColor color, bool isOn);
*/

void ControlLed();

#define LED_TASK_STACK_SIZE 128
#define LED_TASK_PRIORITY tskIDLE_PRIORITY + 2

// Task for manipulating color of rgb led.
int LedTaskInit();
