/*
 * control.c
 *
 *  Created on: May 2, 2015
 *      Author: treww
 */

#include "FreeRTOS.h"
#include "task.h"

#include "control.h"
#include "led_task.h"


#define CONTROL_TASK_STACK_SIZE 128
#define CONTROL_TASK_PRIORITY (tskIDLE_PRIORITY + 1)

static portTASK_FUNCTION(ControlFunc, params)
{
  LedAction action;
  action.Color = Green;
  action.IsOn = 1;
  while(1)
  {
    switch (action.Color)
    {
      case Green:
        action.Color = Red;
        break;
      case Red:
        action.Color = Blue;
        break;
      case Blue:
        action.Color = Green;
        break;
      default:
        while(1);
    }
    EnableLed(&action);
    vTaskDelay(portTICK_PERIOD_MS * 100);

    if (action.Color == Green)
      action.IsOn = action.IsOn ? 0 : 1;
  }
}

void InitControlTask()
{
  if (xTaskCreate(ControlFunc, "SwitchLed", CONTROL_TASK_STACK_SIZE, NULL, CONTROL_TASK_PRIORITY, NULL) != pdTRUE)
  {
    while(1);
  }
}
