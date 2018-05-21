//****************************  PLATFORM SELECTION    ********************************//

#define ARDUINO_PLATFORM    // ( with Atmega168 ) bootloader development on arduino plattform
#define ATMEGA168_MICROCONTROLLER
// #define ARDUINO_DEBUG

//**************************** END PLATFORM DEFINITION ********************************//

/*
  AudioBoot - flashing a microcontroller by PC audio line out 
        This version is with differential manchester coding

  Hardware:   Atmega8 
        Atmega168

  input pin:  should be connected to a voltage divider.

        The input pin is also connected by a 10nF capacitor to the PC line out
        
        The Atmega168 seems to have the switching voltage level at 2.2V
        The Atmega8 at 1.4V
        The switching levels of the input pins may vary a little bit from one
        MC to another.  If you to be able to adjust the voltages, 
        use a 10k poti as voltage divider.

  outuput:  status led connected to a pin with a 470 Ohm resistor


  As developement platform an Arduino Diecimilla was used. Therefore you
  will find many #ifdefs for the Arduino in this code.
  If you want to optimize the bootloader further you may use an Arduino 
  as developement platform.


  necessary setup

  1. Project->ConfigurationOptions->Processortype
  2. Project->ConfigurationOptions->Programming Modell 'Os'
  3. Project->ConfigurationOptions->CustomOptions->LinkerOptions->see further down
  5. BootresetvectorFuseBit enable Bootloader
  6. compile and flash Bootloader


  ATMEGA8, 1K byte bootloader: -Wl,--section-start=.text=0x1c00
  HIGH:0xDA
  LOW:0xE4

  ATMEGA168, 1K bootloader: -Wl,--section-start=.text=0x3c00
  Fuses Atmega168:
  Extended: 0xFA
  HIGH: 0xDF
  LOW: E2


  to protect the bootloader section from beeing overwritten, set the memory protection
  flags as follows:

  BOOTSz1=0;BOOTSz0=0;
  ( lockbits: 0xCF ==> LPM and SPM prohibited in bootloader section )

  ************************************************************************************

  (c) -C-H-R-I-S-T-O-P-H-   -H-A-B-E-R-E-R- 2011

  v0.1  19.6.2008 C. -H-A-B-E-R-E-R-  Bootloader for IR-Interface
  v1.0  03.9.2011 C. -H-A-B-E-R-E-R-  Bootloader for audio signal
  v1.1  05.9.2011 C. -H-A-B-E-R-E-R-  changing pin setup, comments, and exitcounter=3 
  v1.2  12.5.2012 C. -H-A-B-E-R-E-R-  Atmega8 Support added, java programm has to be addeptet too 
  v1.3  20.5.2012 C. -H-A-B-E-R-E-R-  now interrupts of user programm are working 
  v1.4  05.6.2012 C. -H-A-B-E-R-E-R-  signal coding changed to differential manchester code
  v2.0  13.6.2012 C. -H-A-B-E-R-E-R-  setup for various MCs

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
*/
//***************************************************************************************
//** the data transmition was developed on the arduino plattform
//***************************************************************************************


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>


#ifdef ARDUINO_PLATFORM

  // The setup() method runs once, when the sketch starts

  void setup()   {                
  #ifdef ARDUINO_DEBUG
      Serial.begin(9600); 
      Serial.println("AudioBoot starting");
      delay(1000);
  #endif  
  }
  
  void main()                     
  {
    setup();
    a_main();
    while(1){
      
    }
  
  }

  // initialize the digital pin as an output:
	#define LEDPORT    ( 1<<PB0 ); //PB0 RX Led
	#define INITLED    { DDRB|=LEDPORT; }
	#define LEDON      { PORTB|=LEDPORT;}
	#define LEDOFF     { PORTB&=~LEDPORT;}
	#define TOGGLELED  { PORTB^=LEDPORT;}

  //data input pin (audio in)
  #define INPUTAUDIOPIN (1<<PB4)
  #define PINVALUE (PINB&INPUTAUDIOPIN)
  #define PINLOW (PINVALUE==0)
  #define PINHIGH (!PINLOW)

#endif
//***************************************************************************************
// end of arduino specific developement plattform code
//***************************************************************************************
//***************************************************************************************
// here starts the code for the stand alone microcontroller
//***************************************************************************************

#define BOOTLOADER_STARTADDRESS BOOTLOADER_ADDRESS    // start address:
// #define LAST_PAGE (BOOTLOADER_ADDRESS - SPM_PAGESIZE) / SPM_PAGESIZE
#define LAST_PAGE 123

//***************************************************************************************
// main loop
//***************************************************************************************

#define TIMER TCNT0 // we use timer2 for measuring time

// frame format definition
#define COMMAND         0
#define PAGEINDEXLOW    1  // page address lower part
#define PAGEINDEXHIGH   2  // page address higher part
#define LENGTHLOW       3
#define LENGTHHIGH      4
#define CRCLOW          5  // checksum lower part 
#define CRCHIGH         6  // checksum higher part 
#define DATAPAGESTART   7  // start of data
#define PAGESIZE        SPM_PAGESIZE
#define FRAMESIZE       (PAGESIZE+DATAPAGESTART) // size of the data block to be received

// bootloader commands
#define NOCOMMAND       0
#define TESTCOMMAND     1
#define PROGCOMMAND     2
#define RUNCOMMAND      3
#define EEPROMCOMMAND   4
#define EXITCOMMAND     5


uint8_t FrameData[FRAMESIZE];

//***************************************************************************************
// receiveFrame()
//
// This routine receives a differential manchester coded signal at the input pin.
// The routine waits for a toggling voltage level. 
// It automatically detects the transmission speed.
//
// output:    uint8_t flag: true: checksum ok
//        Data // global variable
// 
//***************************************************************************************

uint8_t receiveFrame()
{
  //uint16_t store[16];

  uint16_t counter = 0;
  volatile uint16_t time = 0;
  volatile uint16_t delayTime;
  uint8_t p, t;
  uint8_t k = 8;
  uint8_t dataPointer = 0;
  uint16_t n;

  //*** synchronisation and bit rate estimation **************************
  time = 0;
  // wait for edge
  p = PINVALUE;
  while (p == PINVALUE);

  p = PINVALUE;

  TIMER = 0; // reset timer
  for (n = 0; n < 16; n++)
  {
    // wait for edge
    while (p == PINVALUE);
    t = TIMER;
    TIMER = 0; // reset timer
    p = PINVALUE;

    //store[counter++] = t;

    if (n >= 8)time += t; // time accumulator for mean period calculation only the last 8 times are used
  }

  delayTime = time * 3 / 4 / 8;
  // delay 3/4 bit
  while (TIMER < delayTime);

  //p=1;

  //****************** wait for start bit ***************************
  while (p == PINVALUE) // while not startbit ( no change of pinValue means 0 bit )
  {
    // wait for edge
    while (p == PINVALUE);
    p = PINVALUE;
    TIMER = 0;

    // delay 3/4 bit
    while (TIMER < delayTime);
    TIMER = 0;

    counter++;
  }
  p = PINVALUE;
  
  //****************************************************************
  //receive data bits
  k = 8;
  for (n = 0; n < (FRAMESIZE * 8); n++)
  {
    // wait for edge
    while (p == PINVALUE);
    TIMER = 0;
    p = PINVALUE;

    // delay 3/4 bit
    while (TIMER < delayTime);

    t = PINVALUE;
    

    counter++;

    FrameData[dataPointer] = FrameData[dataPointer] << 1;
    if (p != t) FrameData[dataPointer] |= 1;
    p = t;
    k--;
    if (k == 0) {
      dataPointer++;
      k = 8;
    };
  }
  uint16_t crc = (uint16_t)FrameData[CRCLOW] + FrameData[CRCHIGH] * 256;

  #ifdef ARDUINO_DEBUG
  //********************** debug ********************************

  Serial.print("mean time:");
  Serial.println(time/8);
  Serial.print("3/4 time:");
  Serial.println(delayTime);
  Serial.print("counter:");
  Serial.println(counter);
  Serial.println("*********************");
//  for(n=0;n<FRAMESIZE;n++)
//  {
//    Serial.print(FrameData[n],HEX);
//    Serial.print(" ");
//
//    Serial.println((int)FrameData[n]);
//  } 
  Serial.println("*********************");

  Serial.print("COMMAND:");
  Serial.println((int)FrameData[COMMAND]);
  Serial.print("PAGEINDEX:");
  Serial.println((int) FrameData[PAGEINDEXLOW]+256*FrameData[PAGEINDEXHIGH]);
  Serial.print("CRC:");

  Serial.println(crc,HEX);
  Serial.println("*********************");
#endif
  
  return 1;
}



//***************************************************************************************
//  void boot_program_page (uint32_t page, uint8_t *buf)
//
//  Erase and flash one page.
//
//  inputt:     page address and data to be programmed
// 
//***************************************************************************************
void boot_program_page (uint32_t page, uint8_t *buf)
{
  uint16_t i;

    if (page > LAST_PAGE) return;
    
    cli(); // disable interrupts

    boot_page_erase (page);
    boot_spm_busy_wait ();      // Wait until the memory is erased.

    for (i=0; i<SPM_PAGESIZE; i+=2)
    {
        uint16_t w = *buf++;
        w += (*buf++) << 8;
        
        boot_page_fill (page + i, w);
      boot_spm_busy_wait();       // Wait until the memory is written.
  }

    boot_page_write (page);     // Store buffer in flash page.
    boot_spm_busy_wait();       // Wait until the memory is written.
  
    boot_rww_enable ();
}
//***************************************************************************************
void initstart()
{
   // Timer 2 normal mode, clk/8, count up from 0 to 255  
   // ==> frequency @16MHz= 16MHz/8/256=7812.5Hz
#ifdef ATMEGA168_MICROCONTROLLER
  TCCR0A=0;
  TCCR0B=0;
   TCCR0B= _BV(CS01);
#endif
}

//***************************************************************************************
// main loop
//***************************************************************************************
void a_main()
{
  initstart();
 #ifdef ARDUINO_DEBUG  
  Serial.println("waiting for signal");
  Serial.println(SPM_PAGESIZE);
 #endif

  uint8_t p;
  #define WAITBLINKTIME 10000
  uint16_t time=WAITBLINKTIME;
  uint8_t timeout=6;
 
  //*************** wait for toggling input pin or timeout ******************************
  uint8_t exitcounter=3;
  while(1)
  {  

    if(TIMER>100)  // timedelay ==> frequency @16MHz= 16MHz/8/100=20kHz
    {
       TIMER=0;
       time--;
       if(time==0)
       {
         TOGGLELED;

     time=WAITBLINKTIME;
         timeout--;
         if(timeout==0)
         {

           LEDOFF; // timeout,
           // leave bootloader and run program
       StartSketch();
         }
       }
    }
    if(p!=PINVALUE)
    {
  //setSpalte(3);
      p=PINVALUE;
      exitcounter--;
    }
    if(exitcounter==0) break; // signal received, leave this loop and go on
  }
  //*************** start command interpreter *************************************  
  LEDON;
  while(1)
  {
    if(!receiveFrame())
    {
      //*****  error: blink fast, press reset to restart *******************
      while(1)
      {         
        if(TIMER>100)  // timerstop ==> frequency @16MHz= 16MHz/8/100=20kHz
        {
           TIMER=0;
           time--;
           if(time==0)
           {
             TOGGLELED;
             time=1000;
           }
        }
      }
    }
    else // succeed
    {
      TOGGLELED;
      switch(FrameData[COMMAND])
      {
        case TESTCOMMAND: // not used yet
        {
  #ifdef ARDUINO_DEBUG        
         Serial.println("testcommand");
  #endif
        }
        break;
        case RUNCOMMAND:
        {
  #ifndef ARDUINO_DEBUG
          // leave bootloader and run program
    StartSketch();
  #endif
  #ifdef ARDUINO_DEBUG        
         Serial.println("runcommand");
  #endif
        }
        break;
        case PROGCOMMAND:
        { 
  #ifdef ATMEGA168_MICROCONTROLLER
        // Atmega168 Pagesize=64 Worte=128 Byte
      uint16_t k;
      k=(((uint16_t)FrameData[PAGEINDEXHIGH])<<8)+FrameData[PAGEINDEXLOW];
      boot_program_page (SPM_PAGESIZE*k, FrameData+DATAPAGESTART);  // erase and programm page
  #endif

  #ifdef ARDUINO_DEBUG 
          Serial.println("programming");
  #endif
        }
        break;
      }
      FrameData[COMMAND]=NOCOMMAND; // delete command
    }
  }
}


void StartSketch(void)
{
  cli();

  /* Undo TIMER1 setup and clear the count before running the sketch */
  TCCR0B = 0;
  TIMER = 0;
//    
//  /* Relocate the interrupt vector table to the application section */
//  MCUCR = (1 << IVCE);
//  MCUCR = 0;


  /* jump to beginning of application space */
  __asm__ volatile("jmp 0x0000");
}
