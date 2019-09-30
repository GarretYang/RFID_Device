/******************************************************************************/
/*
implementation for reading from RFID card
*/
/******************************************************************************/

#include <stdint.h>
#include <tm4c123gh6pm.h>
#include "SPI_2_MFRC522.h"

int main()
{
  
  unsigned char FoundTag; // Variable used to check if Tag was found
  unsigned char ReadTag; // Variable used to store anti-collision value to read Tag information
  unsigned char TagData[MAX_LEN]; // Variable used to store Full Tag Data
  unsigned char TagSerialNumber[5]; // Variable used to store only Tag Serial Number
  unsigned char ReadTagPrev; //previous tag read
  
  // debug serial port initialization  
  UART0_Init(16);
  
  // init SPI protocol between Tiva and RC522
  SPI2_Init(); //initialize SPI2 for:
  //PB4 = SCK, PB5 = NSS (SDA), PB6 = MISO, PB7 = MOSI
  
  // control of Reset pin
  PortF_Init(); //PF3 = RST
  GPIO_PORTF_DATA_R = 0;
  resetRFID();
  
  GPIO_PORTF_DATA_R |= 0x2; //cs on PF1
  
  // init MFRC522
  begin();
  
  unsigned char info = getFirmwareVersion();
  //printCharHex(info);
  
  //  int test = digitalSelfTestPass();
  //  for (int i = 0; i < 2; i++);
  ReadTagPrev = 0;
  while(1)
  {
    FoundTag = requestTag(MF1_REQIDL, TagData);
    if (FoundTag == MI_OK) 
    {
      delay(200);
      ReadTag = antiCollision(TagData);
      if (1)
      {
        for(int i = 0; i < 16; i++){
          printCharHex(TagData[i]);
          delay(10000);
        }
        transmit0('\n\r');
        ReadTagPrev = ReadTag;
      } 
      //memcpy(TagSerialNumber, TagData, 4); // Write the Tag information in the TagSerialNumber variable
    }
  }
  return 0;
}
