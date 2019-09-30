#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "drivers/rgb.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"
#include "led_task.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "SSD2119.h"
#include "tivaCbluetooth.c"
#include "FileIO.c"
#include "global.h"

#define PE1_GREEN 0x02
#define PE2_YELLOW 0x04
#define PE3_RED 0x08

//GPIO INITIALIZATIONS
#define RCGCGPIO (*((unsigned int *)0x400FE608)) //location for enabling GPIO
#define RCGC2GPIO (*((unsigned int *)0x400FE108)) //run mode clock gating control

//PortE:
#define GPIO_DIR_PORTE (*((unsigned int *)0x40024400)) //GPIO direction Port E
#define GPIO_DEN_PORTE (*((unsigned int *)0x4002451C)) //digital enable Port E
#define GPIO_LOCK_PORTE (*((unsigned int *)0x40024520)) //unlocking GPIO Port E
#define GPIO_CR_PORTE (*((unsigned int *)0x40024524)) //commiting pins Port E
#define GPIO_PUR_PORTE (*((unsigned int *)0x40024510)) //enabling pull down resistors Port E
#define GPIO_REG_PORTE (*((unsigned int *)0x40024420)) //Alternate Function GPIOAFSEL
#define GPIO_AMSEL_PORTE (*((unsigned int *)0x40024528)) //disable isolation
#define GPIO_DATA_PORTE (*((unsigned int *)0x400243fc)) //port E GPIODATA
#define GPIO_PCTL_PORTE (*((unsigned int *)0x4002452c))

// traffic light led
#define PE1_GREEN 0x02
#define PE2_YELLOW 0x04
#define PE3_RED 0x08

#define STARTSTOP 1
#define PASSENGER 2

//*****************************************************************************
//
// The stack size for the LED toggle task.
//
//*****************************************************************************
#define LEDTASKSTACKSIZE        128         // Stack size in words

//*****************************************************************************
//
// The item size and queue size for the LED message queue.
//
//*****************************************************************************
#define LED_ITEM_SIZE           sizeof(uint8_t)
#define LED_QUEUE_SIZE          128

//*****************************************************************************
//
// Default LED toggle delay value. LED toggling frequency is twice this number.
//
//*****************************************************************************
#define LED_TOGGLE_DELAY        250

void LED_On_PortE(unsigned int led);
void LED_Off_PortE(void);

typedef enum {
  STOP,
  GO,
  PASS,
  INIT
} state;

state STATE;

xQueueHandle g_pTrafficQueue;
extern xSemaphoreHandle g_pUARTSemaphore;
portTickType start;
portTickType last = 0;


//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************
static void
OutputTask(void *pvParameters)
{
  uint8_t i8Message;
  time_t rawtime;
  struct tm * timeinfo;
  
  short xPos = Touch_ReadX();
  short yPos = Touch_ReadY();
  
  LCD_DrawFilledRect(0 ,0 , 100, 40, ((0xFF>>3)<<11) | ((0xFF>>2)<<5) | (0xFF>>3));
  LCD_SetCursor(0,0);
  LCD_PrintInteger((int)xPos);
  LCD_SetCursor(30,0);
  LCD_PrintInteger((int)yPos);
  
  //
  // Loop forever.
  //
  while(1)
  {
    
    //
    // Read the next message, if available on queue.
    //
    LED_On_PortE(PE2_YELLOW);
    if(xQueueReceive(g_pTrafficQueue, &i8Message, 0) == pdPASS)
    {
      if(i8Message == HAS_TAG)
      {
        if (INDEX == -1) {
          writeStringToUart3("tag doesn't exist\r\n");
          LED_On_PortE(PE3_RED);
        } else {
          LED_On_PortE(PE1_GREEN);
          writeStringToUart3("ID: ");
          writeStringToUart3(INFO[INDEX].id);
          writeStringToUart3(" Name: ");
          writeStringToUart3(INFO[INDEX].name);
//          time (&rawtime);
//          timeinfo = localtime (&rawtime);
//          writeStringToUart3(asctime(timeinfo));
          
          writeStringToUart3("\r\n");
          
          
          LOCKED = 1;
          INDEX = -1;
        }
          vTaskDelay(2000); 
      }
      
      //
      // If left button, update to next LED.
      //
      if(i8Message == MOTOR_ON)
      {
        LOCKED = 0; // locked
        writeStringToUart3("system locked\r\n");
        LED_On_PortE(PE2_YELLOW);
      }
    }

  }
  
}

void LED_On_PortE(unsigned int led) {
    GPIO_PORTE_DATA_R = led;
}

void LED_Off_PortE() {
    GPIO_PORTE_DATA_R = 0;
}

//*****************************************************************************
//
// Initializes the LED task.
//
//*****************************************************************************
uint32_t
OutputTaskInit(void)
{
  // Port E
  volatile unsigned long delay;
  RCGC2GPIO |= 0x10;        // activate clock for port E
  delay = RCGC2GPIO;
  
  GPIO_PCTL_PORTE &= ~0xFFF0;  // regular GPIO*
  GPIO_AMSEL_PORTE &= ~0x0E;  // disable analog on PE1, PE2, PE3
  GPIO_DIR_PORTE |= 0x0E;     // set direction to onput
  GPIO_REG_PORTE &= ~0x0E;  // regular port function
  GPIO_DEN_PORTE |= 0x0E;     //enable digital port
  
  g_pTrafficQueue = xQueueCreate(LED_QUEUE_SIZE, sizeof(int));
  
  STATE = INIT;
  start = xTaskGetTickCount();
  
  //
  // Create the LED task.
  //
  if(xTaskCreate(OutputTask, (const portCHAR *)"LED", LEDTASKSTACKSIZE, NULL,
                 tskIDLE_PRIORITY + PRIORITY_TRAFFIC_TASK, NULL) != pdTRUE)
  {
    return(1);
  }
  
  //
  // Success.
  //
  return(0);
}

