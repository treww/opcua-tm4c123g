/*
 * led_task.c
 *
 *  Created on: May 1, 2015
 *      Author: treww
 */

#include "led_task.h"

#include "tm4c123gh6pm.h"

volatile uint32_t ui32Loop;

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
  GPIO_PORTF_DIR_R = 0x08;
  GPIO_PORTF_DEN_R = 0x08;
}


void ControlLed()
{
    //
    // Loop forever.
    //
    while(1)
    {
        //
        // Turn on the LED.
        //
        GPIO_PORTF_DATA_R &= ~(0x08);

        //
        // Delay for a bit.
        //
        for(ui32Loop = 0; ui32Loop < 20000; ui32Loop++)
        {
        }

        //
        // Turn off the LED.
        //
        GPIO_PORTF_DATA_R |= 0x08;

        //
        // Delay for a bit.
        //
        for(ui32Loop = 0; ui32Loop < 20000; ui32Loop++)
        {
        }
    }


}

static portTASK_FUNCTION(SwitchColor, params)
{
  ControlLed();
  vTaskDelete(NULL);
}

static void Hang()
{
  while (1);
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
