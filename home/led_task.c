/*
 * led_task.c
 *
 *  Created on: May 1, 2015
 *      Author: treww
 */

#include "led_task.h"

#include "tm4c123gh6pm.h"
#include "queue.h"

volatile uint32_t ui32Loop;
QueueHandle_t LedQueue;

static void Hang()
{
  while (1);
}

static void InitializeLed()
{
  //
  // Enable the GPIO port that is used for the on-board LED.
  //
  SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;

  //
  // Do a dummy read to insert a few cycles after enabling the peripheral.
  //
  ui32Loop = SYSCTL_RCGC2_R;

  //
  // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
  // enable the GPIO pin for digital function.
  //
  GPIO_PORTF_DIR_R = 0x0E;
  GPIO_PORTF_DEN_R = 0x0E;

  LedQueue = xQueueCreate(10, sizeof(LedAction));
  if (LedQueue == 0)
    Hang();
}


static void TurnOnLed(LedColor color)
{
  GPIO_PORTF_DATA_R &= ~((unsigned int)color);
}

static void TurnOffLed(LedColor color)
{
  GPIO_PORTF_DATA_R |= ((unsigned int)color);
}

void ControlLed()
{
    //
    // Loop forever.
    //
    while(1)
    {

        LedAction action = {0};
        if (!xQueueReceive(LedQueue, &action, portMAX_DELAY))
          continue;

        if (action.IsOn)
          TurnOnLed(action.Color);
        else
          TurnOffLed(action.Color);
    }
}

static portTASK_FUNCTION(SwitchColor, params)
{
  ControlLed();
  vTaskDelete(NULL);
}

void EnableLed(const LedAction* action)
{
  if (xQueueSendToBack(LedQueue, action, portMAX_DELAY) != pdPASS)
    Hang();
}


int LedTaskInit()
{
  InitializeLed();
  if (xTaskCreate(SwitchColor, "SwitchLed", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, NULL) != pdTRUE)
  {
    Hang();
  }
  return 0;
}
