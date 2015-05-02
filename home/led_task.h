
#include "FreeRTOS.h"
#include "task.h"

// Color of led
typedef enum
{
  Red,
  Blue,
  Green
}LedColor;

/*
// Enable/Disable led with specified color
void EnableLed(int8_t color, int8_t isOn);
// Enable/Disable led with specified color

// this method should be called fro ISR.
void EnableLedFromISR(LedColor color, bool isOn);
*/

void ControlLed();

#define LED_TASK_STACK_SIZE 128
#define LED_TASK_PRIORITY tskIDLE_PRIORITY + 1

// Task for manipulating color of rgb led.
int LedTaskInit();
