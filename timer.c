void
TimerInit(void)
{
    //
    // Enable the GPIO port to which the pushbuttons are connected.
    //
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  
  RCGCTIMER |= 0x1;
  GPTMCTL &= ~0x1;  // disables timer
  GPTMCFG = 0x0;
  GPTMTAMR |= (0x2<<0);  // set TAMR field in GPTMTAMR register to periodic mode
  GPTMTAMR &= (0x0<<4);  // enables GPTM Timer to count down
  GPTMTAILR = 0x00F42400 / div; // sets interval to one second
  GPTMIMR |= 0x1;  // enables time out interrupt mask
  EN0 |= (0x1<<19);
  GPTMCTL |= 0x1;  // enables timer 0, start counting
  GPTMICR |= (0x1<<0); // reset timer 0
  PRI4 = (3<<29);
}