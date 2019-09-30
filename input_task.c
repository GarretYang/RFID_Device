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
#include "global.h"
#include "SPI_2_MFRC522.h"
#include "tivaCbluetooth.c"


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
InputTask(void *pvParameters)
{
  uint32_t ui32SwitchDelay = 25;
  portTickType ui16LastTime = xTaskGetTickCount();
  uint8_t ui8Message;
 
  unsigned char FoundTag; // Variable used to check if Tag was found
  unsigned char ReadTag; // Variable used to store anti-collision value to read Tag information
  unsigned char TagData[MAX_LEN]; // Variable used to store Full Tag Data
  unsigned char TagSerialNumber[5]; // Variable used to store only Tag Serial Number
  unsigned char ReadTagPrev; //previous tag read
  ReadTagPrev = 0; 
  begin();
  //resetRFID();
  //
  // Loop forever.
  //
  while(1)
  {
    short xPos = Touch_ReadX();
    short yPos = Touch_ReadY();
    
    LCD_DrawFilledRect(0 ,0 , 100, 40, ((0xFF>>3)<<11) | ((0xFF>>2)<<5) | (0xFF>>3));
    LCD_SetCursor(0,0);
    LCD_PrintInteger((int)xPos);
    LCD_SetCursor(30,0);
    LCD_PrintInteger((int)yPos);
    LCD_SetCursor(0,20);
    if (LOCKED == 1) {
      LCD_PrintString("unlocked");
    } else {
      LCD_PrintString("locked");
    }
    
    FoundTag = requestTag(MF1_REQIDL, TagData);
    
    // if (abs(xPos - 2300) > 200 && abs(yPos - 1800) > 200 && LOCKED == 1) {
    if (abs(xPos - 1700) < 200 && abs(yPos - 1800) < 200) {    
      ui8Message = MOTOR_ON;
    }

//********************************RFID TASK*********************************        

    else if (FoundTag == MI_OK) 
    {      
      delay(200);
      ReadTag = antiCollision(TagData);
      TagData[5] = '\0';
      for (int i = 0; i < LENGTH; i++) {
        if (strcmp(TagData, TAGS[i]) == 0) {
          INDEX = i;
          ReadTagPrev = ReadTag;
          break;
        }
      }
      ui8Message = HAS_TAG;
    } 
//***************************************************************************** 
    
    else {
      ui8Message = NONE;
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
    
    
    vTaskDelay(1000);
  }
}

//*****************************************************************************
//
// Initializes the switch task.
//
//*****************************************************************************
uint32_t
InputTaskInit(void)
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
  if(xTaskCreate(InputTask, (const portCHAR *)"Touch",
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

int cmp(char* c1, char* c2) {
  while(*c2) {
    if (*(c1++) != *(c2++)) {
      return 1;
    }
  }
  return 0;
}
