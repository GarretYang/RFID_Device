//*****************************************************************************
//
// switch_task.c - A simple switch task to process the buttons.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"
#include "switch_task.h"
#include "led_task.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//*****************************************************************************
//
// The stack size for the display task.
//
//*****************************************************************************
#define SLEEPSTACKSIZE        128         // Stack size in words
#define SLEEP                 1

xQueueHandle g_pSLEEPQueue;
uint8_t i8Message;

extern xQueueHandle g_pLEDQueue;
extern xSemaphoreHandle g_pUARTSemaphore;

//*****************************************************************************
//
// This task reads the buttons' state and passes this information to LEDTask.
//
//*****************************************************************************
static void
SleepTask(void *pvParameters)
{ 
  while(1){
    for(int i = 0; i < 3; i++)
    {
      portSUPPRESS_TICKS_AND_SLEEP(5000);
    }
    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("Sleep Mode.\n");
    xSemaphoreGive(g_pUARTSemaphore);
  }
}

//*****************************************************************************
//
// Initializes the switch task.
//
//*****************************************************************************
uint32_t
SleepModeInit(void)
{
  g_pSLEEPQueue = xQueueCreate(5, sizeof(uint8_t));
  //
  // Create the switch task.
  //
  if(xTaskCreate(SleepTask, (const portCHAR *)"Sleep",
                 SLEEPSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_SLEEP_TASK, NULL) != pdTRUE)
  {
    return(1);
  }
  
  //
  // Success.
  //
  return(0);
}
