#include <stdbool.h>
#include <stdint.h>
#include <math.h>
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
#include "SSD2119.h"

// center of the screen (pixels)
#define centerx 160
#define centery 120
#define radius 30

#define STARTSTOP 1
#define PASSENGER 2

//*****************************************************************************
//
// The stack size for the display task.
//
//*****************************************************************************
#define TOUCHTASKSTACKSIZE        128         // Stack size in words

extern xQueueHandle g_pLEDQueue;
extern xQueueHandle g_pTrafficQueue;
extern xSemaphoreHandle g_pUARTSemaphore;

extern int start_b = 0;
extern int pass_b = 0;

portTickType start_touch;

//*****************************************************************************
//
// This task reads the buttons' state and passes this information to LEDTask.
//
//*****************************************************************************
static void
TouchTask(void *pvParameters)
{
  uint32_t ui32SwitchDelay = 25;
  portTickType ui16LastTime = xTaskGetTickCount();
  uint8_t ui8Message;

  //
  // Loop forever.
  //
  while(1)
  {
    short xPos = Touch_ReadX();
    short yPos = Touch_ReadY();
    
    //        if(xPos < 0) xPos = 0;
    //        if(yPos < 0) yPos = 0;
    
    LCD_DrawFilledRect(0 ,0 , 100, 40, ((0xFF>>3)<<11) | ((0xFF>>2)<<5) | (0xFF>>3));
    LCD_SetCursor(0,0);
    LCD_PrintInteger((int)xPos);
    LCD_SetCursor(30,0);
    LCD_PrintInteger((int)yPos);
      
//    if (abs(xPos - 0) > 100 && abs(yPos - 1800) > 100) { 

      // stop / start
      if (abs(xPos - 1800) < 200 && abs(yPos - 1800) < 200) {
              LCD_DrawFilledRect(50, 0 , 20, 20, ((0xFF>>3)<<11) | ((0xFF>>2)<<5) | (0xFF>>3));
        LCD_SetCursor(50,0);
        LCD_PrintString("stop");
        if (xTaskGetTickCount() - start_touch > 2000) {
          ui8Message = STARTSTOP;
          start_touch = xTaskGetTickCount();
        }        
      } else if (abs(xPos - 800) < 200 && abs(yPos - 1800) < 200) { 
        LCD_DrawFilledRect(50, 0 , 20, 20, ((0xFF>>3)<<11) | ((0xFF>>2)<<5) | (0xFF>>3));
        LCD_SetCursor(50,0);
        LCD_PrintString("passenger");
        if (xTaskGetTickCount() - start_touch > 2000) {
          ui8Message = PASSENGER;
          start_touch = xTaskGetTickCount();
        }
      } else {
        ui8Message = 0;
        start_touch = xTaskGetTickCount();
      }
      
      //
      // Pass the value of the button pressed to LEDTask.
      //
      if(xQueueSend(g_pTrafficQueue, &ui8Message, portMAX_DELAY) !=
         pdPASS)
      {
        //
        // Error. The queue should never be full. If so print the
        // error message on UART and wait for ever.
        //
        UARTprintf("\nQueue full. This should never happen.\n");
        while(1)
        {
        }
      }
      
      
//    } else {
//      start_touch = xTaskGetTickCount();
//    }
    

    vTaskDelay(100);
  }
}

//*****************************************************************************
//
// Initializes the switch task.
//
//*****************************************************************************
uint32_t
TouchTaskInit(void)
{
  
  //
  // Initialize touch
  //
  Touch_Init();
  
  //draw Buttons
  LCD_DrawFilledCircle(centerx - 90, centery + 40, radius, ((0xFF>>3)<<11) | ((0x55>>2)<<5) | (0x55>>3));
  LCD_DrawCircle(centerx - 90, centery + 40, radius, 0);
  LCD_DrawFilledCircle(centerx + 90, centery + 40, radius, ((0x55>>3)<<11) | ((0x55>>2)<<5) | (0xFF>>3));
  LCD_DrawCircle(centerx + 90, centery + 40, radius, 0);
  
  //
  // Create the switch task.
  //
  if(xTaskCreate(TouchTask, (const portCHAR *)"Touch",
                 TOUCHTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_TOUCH_TASK, NULL) != pdTRUE)
  {
    return(1);
  }
  
  //
  // Success.
  //
  return(0);
}
