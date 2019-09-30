/**************************************************************************/
/*
file    SPI_2_MFRC522.c

communication and protocols for TM and MFRC522
*/
/**************************************************************************/

#include <stdint.h>
#include "SPI_2_MFRC522.h"
#include <tm4c123gh6pm.h>

/**************************************************************************/
/*!

@brief   Writes value to a register.

@param   addr  The address a register.
@param   val   The value to write to a register.

*/
/**************************************************************************/
void writeToRegister(unsigned char addr, unsigned char val) {
  GPIO_PORTF_DATA_R &= ~0x2; //cs on PF1
  
  //Address format: 0XXXXXX0
  SPI_ReadWrite((addr<<1)&0x7E);
  SPI_ReadWrite(val);
  
  for(int i = 0; i < 15; i++);
  GPIO_PORTF_DATA_R |= 0x2; //cs on PF1
}

/**************************************************************************/
/*!

@brief   Reads the value at a register.

@param   addr  The address a register.

@returns The byte at the register.

*/
/**************************************************************************/
unsigned char readFromRegister(unsigned char addr) {
  unsigned char val;

  GPIO_PORTF_DATA_R &= ~0x2; //cs on PF1
  
  SPI_ReadWrite(((addr<<1)&0x7E) | 0x80);
  val = SPI_ReadWrite(0x00);
  
  for(int i = 0; i < 15; i++);
  GPIO_PORTF_DATA_R |= 0x2; //cs on PF1
  
  return val;
}

/**************************************************************************/
/*!

@brief   Adds a bitmask to a register.

@param   addr   The address a register.
@param   mask  The mask to update the register with.

*/
/**************************************************************************/
void setBitMask(unsigned char addr, unsigned char mask) {
  unsigned char current;
  current = readFromRegister(addr);
  writeToRegister(addr, current | mask);
}

/**************************************************************************/
/*!

@brief   Removes a bitmask from the register.

@param   reg   The address a register.
@param   mask  The mask to update the register with.

*/
/**************************************************************************/
void clearBitMask(unsigned char addr, unsigned char mask) {
  unsigned char current;
  current = readFromRegister(addr);
  writeToRegister(addr, current & (~mask));
}

/**************************************************************************/
/*!

@brief   Does the setup for the MFRC522.

*/
/**************************************************************************/
void begin() {
  
  reset();
  
  //Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
  writeToRegister(TModeReg, 0x8D);       // Tauto=1; f(Timer) = 6.78MHz/TPreScaler
  writeToRegister(TPrescalerReg, 0x3E);  // TModeReg[3..0] + TPrescalerReg
  writeToRegister(TReloadRegL, 30);
  writeToRegister(TReloadRegH, 0);
  
  writeToRegister(TxAutoReg, 0x40);      // 100%ASK
  writeToRegister(ModeReg, 0x3D);        // CRC initial value 0x6363
  
  setBitMask(TxControlReg, 0x03);        // Turn antenna on.
}

/**************************************************************************/
/*!

@brief   Sends a SOFTRESET command to the MFRC522 chip.

*/
/**************************************************************************/
void reset() {
  writeToRegister(CommandReg, MFRC522_SOFTRESET);
}

/**************************************************************************/
/*!

@brief   Checks the firmware version of the chip.

@returns The firmware version of the MFRC522 chip.

*/
/**************************************************************************/
unsigned char getFirmwareVersion() {
  unsigned char response;
  response = readFromRegister(VersionReg);
  return response;
}

/**************************************************************************/
/*!

@brief   Runs the digital self test.

@returns True if the self test passes, false otherwise.

*/
/**************************************************************************/
int digitalSelfTestPass() {
  int i;
  unsigned char n;

  unsigned char selfTestResultV1[] = {0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
                          0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
                          0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
                          0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
                          0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
                          0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
                          0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
                          0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79};
  unsigned char selfTestResultV2[] = {0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
                          0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
                          0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
                          0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
                          0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
                          0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
                          0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
                          0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F};
  unsigned char *selfTestResult;
  switch(getFirmwareVersion()) {
    case 0x91 :
      selfTestResult = selfTestResultV1;
      break;
    case 0x92 :
      selfTestResult = selfTestResultV2;
      break;
    default:
      return 0;
  }

  reset();
  writeToRegister(FIFODataReg, 0x00);
  writeToRegister(CommandReg, MFRC522_MEM);
  writeToRegister(AutoTestReg, 0x09);
  writeToRegister(FIFODataReg, 0x00);
  writeToRegister(CommandReg, MFRC522_CALCCRC);

  // Wait for the self test to complete.
  i = 0xFF;
  do {
    n = readFromRegister(DivIrqReg);
    i--;
  } while ((i != 0) && !(n & 0x04));

  for (i=0; i < 64; i++) {
    if (readFromRegister(FIFODataReg) != selfTestResult[i]) {
      // Serial.println(i);
      return 0;
    }
  }
  return 1;
}

/**************************************************************************/
/*!

@brief   Sends a command to a tag.

@param   cmd     The command to the MFRC522 to send a command to the tag.
@param   data    The data that is needed to complete the command.
@param   dlen    The length of the data.
@param   result  The result returned by the tag.
@param   rlen    The number of valid bits in the resulting value.

@returns Returns the status of the calculation.
MI_ERR        if something went wrong,
MI_NOTAGERR   if there was no tag to send the command to.
MI_OK         if everything went OK.

*/
/**************************************************************************/
int commandTag(unsigned char cmd, unsigned char *data, int dlen, unsigned char *result, int *rlen) {
  int status = MI_ERR;
  unsigned char irqEn = 0x00;
  unsigned char waitIRq = 0x00;
  unsigned char lastBits, n;
  int i;
  
  switch (cmd) {
  case MFRC522_AUTHENT:
    irqEn = 0x12;
    waitIRq = 0x10;
    break;
  case MFRC522_TRANSCEIVE:
    irqEn = 0x77;
    waitIRq = 0x30;
    break;
  default:
    break;
  }
  
  writeToRegister(CommIEnReg, irqEn|0x80);    // interrupt request
  clearBitMask(CommIrqReg, 0x80);             // Clear all interrupt requests bits.
  setBitMask(FIFOLevelReg, 0x80);             // FlushBuffer=1, FIFO initialization.
  
  writeToRegister(CommandReg, MFRC522_IDLE);  // No action, cancel the current command.
  
  // Write to FIFO
  for (i=0; i < dlen; i++) {
    writeToRegister(FIFODataReg, data[i]);
  }
  
  // Execute the command.
  writeToRegister(CommandReg, cmd);
  if (cmd == MFRC522_TRANSCEIVE) {
    setBitMask(BitFramingReg, 0x80);  // StartSend=1, transmission of data starts
  }
  
  // Waiting for the command to complete so we can receive data.
  i = 25; // Max wait time is 25ms.
  do {
    delay(1);
    // CommIRqReg[7..0]
    // Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
    n = readFromRegister(CommIrqReg);
    i--;
  } while ((i!=0) && !(n&0x01) && !(n&waitIRq));
  
  clearBitMask(BitFramingReg, 0x80);  // StartSend=0
  
  if (i != 0) { // Request did not time out.
    if(!(readFromRegister(ErrorReg) & 0x1D)) {  // BufferOvfl Collerr CRCErr ProtocolErr
      status = MI_OK;
      if (n & irqEn & 0x01) {
        status = MI_NOTAGERR;
      }
      
      if (cmd == MFRC522_TRANSCEIVE) {
        n = readFromRegister(FIFOLevelReg);
        lastBits = readFromRegister(ControlReg) & 0x07;
        if (lastBits) {
          *rlen = (n-1)*8 + lastBits;
        } else {
          *rlen = n*8;
        }
        
        if (n == 0) {
          n = 1;
        }
        
        if (n > MAX_LEN) {
          n = MAX_LEN;
        }
        
        // Reading the recieved data from FIFO.
        for (i=0; i<n; i++) {
          result[i] = readFromRegister(FIFODataReg);
        }
      }
    } else {
      status = MI_ERR;
    }
  }
  return status;
}

/**************************************************************************/
/*!

@brief   Checks to see if there is a tag in the vicinity.

@param   mode  The mode we are requsting in.
@param   type  If we find a tag, this will be the type of that tag.
0x4400 = Mifare_UltraLight
0x0400 = Mifare_One(S50)
0x0200 = Mifare_One(S70)
0x0800 = Mifare_Pro(X)
0x4403 = Mifare_DESFire

@returns Returns the status of the request.
MI_ERR        if something went wrong,
MI_NOTAGERR   if there was no tag to send the command to.
MI_OK         if everything went OK.

*/
/**************************************************************************/
int requestTag(unsigned char mode, unsigned char *data) {
  int status, len;
  writeToRegister(BitFramingReg, 0x07);  // TxLastBists = BitFramingReg[2..0]
  
  data[0] = mode;
  status = commandTag(MFRC522_TRANSCEIVE, data, 1, data, &len);
  
  if ((status != MI_OK) || (len != 0x10)) {
    status = MI_ERR;
  }
  
  return status;
}

/**************************************************************************/
/*!

@brief   Handles collisions that might occur if there are multiple
tags available.

@param   serial  The serial nb of the tag.

@returns Returns the status of the collision detection.
MI_ERR        if something went wrong,
MI_NOTAGERR   if there was no tag to send the command to.
MI_OK         if everything went OK.

*/
/**************************************************************************/
int antiCollision(unsigned char *serial) {
  int status, i, len;
  unsigned char check = 0x00;
  
  writeToRegister(BitFramingReg, 0x00);  // TxLastBits = BitFramingReg[2..0]
  
  serial[0] = MF1_ANTICOLL;
  serial[1] = 0x20;
  status = commandTag(MFRC522_TRANSCEIVE, serial, 2, serial, &len);
  len = len / 8; // len is in bits, and we want each unsigned char.
  if (status == MI_OK) {
    // The checksum of the tag is the ^ of all the values.
    for (i = 0; i < len-1; i++) {
      check ^= serial[i];
    }
    // The checksum should be the same as the one provided from the
    // tag (serial[4]).
    if (check != serial[i]) {
      status = MI_ERR;
    }
  }
  
  return status;
}

/**************************************************************************/
/*!

@brief   Calculates the CRC value for some data that should be sent to
a tag.

@param   data    The data to calculate the value for.
@param   len     The length of the data.
@param   result  The result of the CRC calculation.

*/
/**************************************************************************/
void calculateCRC(unsigned char *data, int len, unsigned char *result) {
  int i;
  unsigned char n;
  
  clearBitMask(DivIrqReg, 0x04);   // CRCIrq = 0
  setBitMask(FIFOLevelReg, 0x80);  // Clear the FIFO pointer
  
  //Writing data to the FIFO.
  for (i = 0; i < len; i++) {
    writeToRegister(FIFODataReg, data[i]);
  }
  writeToRegister(CommandReg, MFRC522_CALCCRC);
  
  // Wait for the CRC calculation to complete.
  i = 0xFF;
  do {
    n = readFromRegister(DivIrqReg);
    i--;
  } while ((i != 0) && !(n & 0x04));  //CRCIrq = 1
  
  // Read the result from the CRC calculation.
  result[0] = readFromRegister(CRCResultRegL);
  result[1] = readFromRegister(CRCResultRegM);
}

/**************************************************************************/
/*!

@brief   Selects a tag for processing.

@param   serial  The serial number of the tag that is to be selected.

@returns The SAK response from the tag.

*/
/**************************************************************************/
unsigned char selectTag(unsigned char *serial) {
  int i, status, len;
  unsigned char sak;
  unsigned char buffer[9];
  
  buffer[0] = MF1_SELECTTAG;
  buffer[1] = 0x70;
  for (i = 0; i < 5; i++) {
    buffer[i+2] = serial[i];
  }
  calculateCRC(buffer, 7, &buffer[7]);
  
  status = commandTag(MFRC522_TRANSCEIVE, buffer, 9, buffer, &len);
  
  if ((status == MI_OK) && (len == 0x18)) {
    sak = buffer[0];
  }
  else {
    sak = 0;
  }
  
  return sak;
}

/**************************************************************************/
/*!

@brief   Handles the authentication between the tag and the reader.

@param   mode    What authentication key to use.
@param   block   The block that we want to read.
@param   key     The authentication key.
@param   serial  The serial of the tag.

@returns Returns the status of the collision detection.
MI_ERR        if something went wrong,
MI_OK         if everything went OK.

*/
/**************************************************************************/
int authenticate(unsigned char mode, unsigned char block, unsigned char *key, unsigned char *serial) {
  int i, status, len;
  unsigned char buffer[12];
  
  //Verify the command block address + sector + password + tag serial number
  buffer[0] = mode;          // 0th byte is the mode
  buffer[1] = block;         // 1st byte is the block to address.
  for (i = 0; i < 6; i++) {  // 2nd to 7th byte is the authentication key.
    buffer[i+2] = key[i];
  }
  for (i = 0; i < 4; i++) {  // 8th to 11th byte is the serial of the tag.
    buffer[i+8] = serial[i];
  }
  
  status = commandTag(MFRC522_AUTHENT, buffer, 12, buffer, &len);
  
  if ((status != MI_OK) || (!(readFromRegister(Status2Reg) & 0x08))) {
    status = MI_ERR;
  }
  
  return status;
}

/**************************************************************************/
/*!

@brief   Tries to read from the current (authenticated) tag.

@param   block   The block that we want to read.
@param   result  The resulting value returned from the tag.

@returns Returns the status of the collision detection.
MI_ERR        if something went wrong,
MI_OK         if everything went OK.

*/
/**************************************************************************/
int readFromTag(unsigned char block, unsigned char *result) {
  int status, len;
  
  result[0] = MF1_READ;
  result[1] = block;
  calculateCRC(result, 2, &result[2]);
  status = commandTag(MFRC522_TRANSCEIVE, result, 4, result, &len);
  
  if ((status != MI_OK) || (len != 0x90)) {
    status = MI_ERR;
  }
  
  return status;
}

/**************************************************************************/
/*!

@brief   Tries to write to a block on the current tag.

@param   block  The block that we want to write to.
@param   data   The data that we shoudl write to the block.

@returns Returns the status of the collision detection.
MI_ERR        if something went wrong,
MI_OK         if everything went OK.

*/
/**************************************************************************/
int writeToTag(unsigned char block, unsigned char *data) {
  int status, i, len;
  unsigned char buffer[18];
  
  buffer[0] = MF1_WRITE;
  buffer[1] = block;
  calculateCRC(buffer, 2, &buffer[2]);
  status = commandTag(MFRC522_TRANSCEIVE, buffer, 4, buffer, &len);
  
  if ((status != MI_OK) || (len != 4) || ((buffer[0] & 0x0F) != 0x0A)) {
    status = MI_ERR;
  }
  
  if (status == MI_OK) {
    for (i = 0; i < 16; i++) {
      buffer[i] = data[i];
    }
    calculateCRC(buffer, 16, &buffer[16]);
    status = commandTag(MFRC522_TRANSCEIVE, buffer, 18, buffer, &len);
    
    if ((status != MI_OK) || (len != 4) || ((buffer[0] & 0x0F) != 0x0A)) {
      status = MI_ERR;
    }
  }
  
  return status;
}

/**************************************************************************/
/*!

@brief   Sends a halt command to the current tag.

@returns Returns the result of the halt.
MI_ERR        If the command didn't complete properly.
MI_OK         If the command completed.
*/
/**************************************************************************/
int haltTag() {
  int status, len;
  unsigned char buffer[4];
  
  buffer[0] = MF1_HALT;
  buffer[1] = 0;
  calculateCRC(buffer, 2, &buffer[2]);
  status = commandTag(MFRC522_TRANSCEIVE, buffer, 4, buffer, &len);
  clearBitMask(Status2Reg, 0x08);  // turn off encryption
  return status;
}



// =============================

void UART0_Init(int mhz)
{
  //enabling clocks
  SYSCTL_RCGCUART_R  |= 0x01; //enables clock on UART0
  for(int i = 0; i < 100; i ++);
  SYSCTL_RCGCGPIO_R |= 0x01; //enables GPIO clock on portA
  for(int i = 0; i < 100; i ++);
  
  UART0_CTL_R &= ~0x00000001; //disable UART0
  if (mhz == 80) {
    UART0_IBRD_R = 520;
    UART0_FBRD_R = 54;
  }
  if (mhz == 16) {
    UART0_IBRD_R = 325;
    UART0_FBRD_R = 25;
  }
  if (mhz == 4) {
    UART0_IBRD_R = 26;
    UART0_FBRD_R = 3;
  }
  UART0_CC_R = 0;
  UART0_LCRH_R = 0x00000060;
  UART0_CTL_R |= 0x00000300; //enable transmit and recieve UART0
  UART0_CTL_R |= 0x00000001; //enable UART0
  
  //setting up GPIO
  GPIO_PORTA_AMSEL_R &= ~0x03; // disable analog
  GPIO_PORTA_AFSEL_R |= 0x03; //alternate hardware functions
  GPIO_PORTA_DEN_R |= 0x03;
  GPIO_PORTA_DIR_R = 0x2;
  
  GPIO_PORTA_DR2R_R &= ~0x03;
  GPIO_PORTA_DR4R_R &= ~0x03;
  GPIO_PORTA_DR8R_R |= 0x03;
  GPIO_PORTA_SLR_R |= 0x03;
  GPIO_PORTA_PCTL_R |= 0x11; // configure PA0 and PA1 pins for UART functions
  while(GPIO_PORTA_DATA_R&0x2 != 0x2);
  GPIO_PORTA_DATA_R = 0;
}

void SPI1_Init(void)
{
  SYSCTL_RCGCSSI_R |= (1<<1);       //enable SSI1
  SYSCTL_RCGCGPIO_R |= (1<<3);      //enable GPIO Port D
  GPIO_PORTD_AFSEL_R |= (1<<0)|(1<<1)|(1<<2)|(1<<3); //alternate functions
  GPIO_PORTD_PCTL_R = 0x00002222; 
  GPIO_PORTD_DEN_R |= (1<<0)|(1<<1)|(1<<2)|(1<<3); 
  GPIO_PORTD_PUR_R |= (1<<0)|(1<<1)|(1<<2)|(1<<3); 
  
  SSI1_CR1_R = 0; //set as master
  SSI1_CC_R = 0; //set primary clock
  SSI1_CPSR_R = 200; //64 division
  SSI1_CR0_R = 0x7; //8bit data
  SSI1_CR1_R |= (1<<1); //enable SSI
}

void PortF_Init(void)
{
  SYSCTL_RCGC2_R |= 0x20;//enables GPIO clock on port
  GPIO_PORTF_LOCK_R = 0x4C4F434B;
  GPIO_PORTF_CR_R = 0xff;
  GPIO_PORTF_DIR_R = 0xFF;  //set port F as output
  GPIO_PORTF_DEN_R = 0xff;  //enables digital PORT F
}

void transmit0(char data)
{
  while(UART0_FR_R&0x0020 != 0);
  UART0_DR_R = data;
  while(UART0_FR_R&0x0008 != 0);
}

void resetRFID(void)
{
  GPIO_PORTF_DATA_R &= ~0x8;
  for(int i = 0; i < 100; i++);
  GPIO_PORTF_DATA_R |= 0x8;
}

void delay(int time)
{
  for(int i = 0; i < time; i++);
}

unsigned char SPI_ReadWrite(unsigned char data)
{
  while((SSI1_SR_R&SSI_SR_TFE) == 0){};				// wait until Tx FIFO empty
  SSI1_DR_R = data;												    // push data out
  while((SSI1_SR_R&SSI_SR_RNE) == 0){};				// wait for response - Rx FIFO
  return (unsigned char)SSI1_DR_R;						// if response is expected to return useful data	
}

char* printCharHex(unsigned char info){
  unsigned char byte1 = ((info>>4) & 0x0F);
  unsigned char byte2 = (info & 0x0F);
  if(byte1 > 0x9){
    byte1 = byte1 + 7;
  }
  if(byte2 > 0x9){
    byte2 = byte2 + 7;
  }
  char newChar[2];
  newChar[0] = (byte1 + '0');
  newChar[1] = (byte2 + '0');
  return newChar;
}


