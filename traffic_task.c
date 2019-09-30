#include <stdbool.h>
#include <stdint.h>
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
#define LED_QUEUE_SIZE          5

//*****************************************************************************
//
// Default LED toggle delay value. LED toggling frequency is twice this number.
//
//*****************************************************************************
#define LED_TOGGLE_DELAY        250

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

extern void LED_On_PortE(unsigned int color) {
  GPIO_DATA_PORTE = 0;
  GPIO_DATA_PORTE |= color;
}

extern void LED_Off_PortE(void) {
  GPIO_DATA_PORTE = 0;
}

//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************
static void
TrafficTask(void *pvParameters)
{
  uint8_t i8Message;
  
  
  
  //
  // Loop forever.
  //
  while(1)
  {
    if (STATE == STOP) { 
      LED_Off_PortE();
      LED_On_PortE(PE3_RED);
      if (xTaskGetTickCount() - start >= 5000) {
        STATE = GO;
        start = xTaskGetTickCount();
      }
    } else if (STATE == GO) { 
      LED_Off_PortE();
      LED_On_PortE(PE1_GREEN);
      if (xTaskGetTickCount() - start >= 5000) {
        STATE = STOP;
        start = xTaskGetTickCount();
      }
    } else if (STATE == PASS) {
      LED_Off_PortE();
      LED_On_PortE(PE2_YELLOW);
      if (xTaskGetTickCount() - start >= 5000) {
        STATE = STOP;
        start = xTaskGetTickCount();
      }
    } else if (STATE == INIT) {
      LED_Off_PortE();
    }
    //
    // Read the next message, if available on queue.
    //
    if(xQueueReceive(g_pTrafficQueue, &i8Message, 0) == pdPASS)
    {
      if (i8Message == STARTSTOP) {
        if (last == 0 || xTaskGetTickCount() - last < 5000) {
          last = xTaskGetTickCount();
        } else {
          if (STATE != INIT) {
            STATE = INIT;
          } else {
            STATE = STOP;
            start = xTaskGetTickCount();
          }
        }
      }
      if (i8Message == PASSENGER) {
        STATE = PASS;
        start = xTaskGetTickCount();
      }
    }
    
    
    
    
    
    
    
    
  }
  
}

//*****************************************************************************
//
// Initializes the LED task.
//
//*****************************************************************************
uint32_t
TrafficTaskInit(void)
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
  if(xTaskCreate(TrafficTask, (const portCHAR *)"LED", LEDTASKSTACKSIZE, NULL,
                 tskIDLE_PRIORITY + PRIORITY_TRAFFIC_TASK, NULL) != pdTRUE)
  {
    return(1);
  }
  
  //
  // Success.
  //
  return(0);
}

