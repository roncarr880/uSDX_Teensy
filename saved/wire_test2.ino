// Testing Teensy 3.2 version I2C functions.  Ignore comments about interrupts.


// Testing a modified version of LCD_Basic for OLED1306 128x64 I2C display. Text based display.
// When just using TWI interrupts the polling state machine hangs in state 3, waiting for stop to clear.
// When the stop is sent, twint does not return to high and no interrupt is generated, hence the twi system
// is not serviced via interrupt in this case.  It continues only when new data is queued via the kick in i2send()
// 
// So it would seem that a call to poll is needed in loop or the timer poll needs to be used.
// The TWI and timer polling could both be used for a system that is quick or the timer polling could be used alone.


// LCD5110_ViewFont 
// Copyright (C)2015 Rinky-Dink Electronics, Henning Karlsen. All right reserved
// web: http://www.RinkyDinkElectronics.com/
//
// This program is a demo of the included full font.
//
// (This program requires a Nokia 5110 LCD module.)
//  Used to be but now has been changed to OLED 1305 spi.
//  and changed again to OLED 1306 I2C
//

// #include <Arduino.h>
// #include <avr/interrupt.h>
#include <OLED1306_Basic.h>
#include <Wire.h>

#define ROW0 0
#define ROW1 8
#define ROW2  16
#define ROW3  24
#define ROW4  32
#define ROW5  40
#define ROW6  48
#define ROW7  56

  OLED1306 myGLCD;

extern unsigned char SmallFont[];
extern unsigned char MediumNumbers[];
extern unsigned char BigNumbers[];

uint8_t display_dirty;


#define I2TBUFSIZE 128             // size power of 2. Size 128 can buffer a full row
#define I2RBUFSIZE 16              // set to expected #of reads in row, power of 2
#define I2INT_ENABLED 1            // 0 for polling in loop or timer, 1 for TWI interrupts

//  I2C buffers and indexes
//unsigned int i2buf[I2TBUFSIZE];   // writes
//uint8_t i2rbuf[I2RBUFSIZE];       // reads
//volatile uint8_t i2in,i2out;
//volatile uint8_t i2rin,i2rout;
volatile uint8_t  gi2state;

unsigned long ints, waits;

void setup()
{

  Serial.begin(38400);
  pinMode(13,OUTPUT);     // testing 
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);

  // Timer0 is already used for millis() - interrupt somewhere in middle
 // OCR0A = 0xAF;
 // TIMSK0 |= _BV(OCIE0A);
 // Serial.println("Non-buffered TWI interrupt test");
}

// TWI interrupt version
// if twi interrupts enabled, then need a handler
// Stop does not produce an interrupt, so need to also poll in loop or via the timer 
//ISR(TWI_vect){
//  i2poll();
//  if( gi2state == 0 ) i2poll();   // needed to get out of state zero, else double ints
//  ++ints;
//}

// timer polling.  about 1000 cps or 8k baud
//ISR(TIMER0_COMPA_vect){
//   i2poll();
//}

void loop()
{
  int i;
  ints = 0; waits = 0;
  
  myGLCD.setFont(SmallFont);
  myGLCD.clrScr();
  myGLCD.print("Hello    ",0,ROW1);
   i2flush();delay(1000);
  myGLCD.print("Upper case: p0r0", LEFT, ROW0);
  myGLCD.print("ABCDEFGHIJKLM", CENTER, ROW2);
  myGLCD.print("NOPQRSTUVWXYZ", CENTER, ROW3);
  myGLCD.print("  Page 1  Write ", CENTER, ROW5);
   i2flush();delay(1000);

  myGLCD.clrScr();
  myGLCD.print("Numbers:", LEFT, ROW0);
  myGLCD.setFont(MediumNumbers);
  myGLCD.print("0123456789", CENTER, ROW2);
  myGLCD.setFont(SmallFont);
   i2flush();delay (1000);
  myGLCD.clrRow(5);
  myGLCD.print("Lower case: p1r0", LEFT, ROW4);  // write to hidden page, not hidden on 128x64
  myGLCD.print("abcdefghijklm", CENTER, ROW6);
  myGLCD.print("nopqrstuvwxyz", CENTER, ROW7);
   i2flush();delay ( 1000 );
  
  myGLCD.clrScr();
  myGLCD.print("Big Numbers:", LEFT, ROW0);
  myGLCD.setFont(BigNumbers);
  myGLCD.print("012345678", LEFT, ROW2);
  myGLCD.setFont(SmallFont);
   i2flush();delay (1000);
  
  myGLCD.clrScr();
  myGLCD.print("Special:", LEFT, ROW0);
  myGLCD.print("!\"#$%&'()*+,-.End", RIGHT, ROW2);
  myGLCD.print("/:;<=>?@[\\]^_`End", RIGHT, ROW3);
   i2flush();delay (1000);
  
  display_dirty = 6;
  freq_display( 3582000 );
   i2flush();delay( 1000 );
  freq_display( 7185000 );
   i2flush();delay( 1000 );
  freq_display( 10106800 );
   i2flush();delay( 1000 );
  freq_display( 14285000 );
   i2flush();delay( 1000 );
  freq_display( 21010234 );
   i2flush();delay( 1000 );
  freq_display( 28345020 );
   i2flush();delay( 1000 );
  freq_display( 5000130 );
   i2flush();delay( 1000 );
  
  // try the new functions
  
  myGLCD.clrScr();
  myGLCD.gotoRowCol(1,5);
  
  for( i = 1; i <= 0xff; ){
     myGLCD.write(i,2);  // bars 
     myGLCD.write(0);    // with spaces
     i <<= 1;
     i |= 1;
  }  
  myGLCD.putch('S');
  
  myGLCD.gotoRowCol(3,4);
char buf[12]; 
String st = "Hello";
  st.toCharArray( buf,6 );
  myGLCD.puts(buf);
 // myGLCD.puts(st.c_str());  // pointer seems to be in program rom instead of ram ??
   i2flush();delay(1000);

  myGLCD.clrScr();
  freq_display( 14285000 );
  
  myGLCD.gotoRowCol(1,0);
  myGLCD.putch('S');
  myGLCD.write(0);
  for( i = 0x80; i < 0xff; ){
     myGLCD.write(i,2);  // bars 
     myGLCD.write(0);    // with spaces
     i >>= 1;
     i |= 0x80;
  }  
  myGLCD.print("USB",32,ROW0);
  myGLCD.print("QST DE W1AW - TEXT IS",0,ROW5);
   i2flush();delay( 3000 );
 

 myGLCD.printNumI(ints,0,ROW7);
 myGLCD.printNumI(waits,CENTER,ROW7);
 i2flush();
 delay(1000);

 Serial.print("Waits..");  Serial.print(waits);
 Serial.print("  Ints.."); Serial.println(ints);
}


void freq_display( unsigned long val ){
int x;

      //myGLCD.clrRow(1,0,127);
      //myGLCD.clrRow(2,0,127);
 
     if( display_dirty & 6 ){       // clean up after alternate use of the display
       myGLCD.clrRow(3,0,127);
       myGLCD.clrRow(2,0,127);
       display_dirty &= ( 0x39 );
     }

      myGLCD.setFont(MediumNumbers);
    //  x= (val >= 10000000 ) ? 0 : 12 ;
       x= 4 *12;
      myGLCD.printNumI(val/1000,x,ROW2,5,'/');    // space character not correct ??
      myGLCD.setFont(SmallFont);
      myGLCD.printNumI((val)%1000,9*12,ROW2,3,'0');

     // LCD.setFont(SmallFont);  redundant

}

/*  I2C write only implementation using polling of the hardware registers */
/*  Teensy 3.2 version  */
/*  most functions do not block */
/*  call i2poll() in loop() to keep everything going */

// use some upper bits in the buffer for control
#define ISTART 0x100
#define ISTOP  0x200
#define I2BUFSIZE 256      
uint16_t i2buf[I2BUFSIZE];
int i2in, i2out;

void i2init(){
    // start up our local I2C routines
  Wire.begin();              // this seems to be the easiest way to get the Pin Mux assigned
  Wire.setClock(400000);     // I2C0_F = 40 : 100k ,  25 = 400k
  //I2C0_F = 15;             // override standard speeds    
}

void i2start( unsigned char adr ){
unsigned int dat;
  // shift the address over and add the start flag
  dat = ( adr << 1 ) | ISTART;
  i2send( dat );
}

void i2send( unsigned int data ){   // just save stuff in the buffer
int next;

  next = (i2in + 1) & (I2BUFSIZE - 1);
  while( i2out == next ) i2poll();
  i2buf[i2in++] = data;
  i2in &= (I2BUFSIZE - 1);
  //i2poll();
}

void i2stop( ){
   i2send( ISTOP );   // que a stop condition
}


void i2flush(){  // needed during setup, call poll to empty out the buffer.  This one does block.

  while( i2poll() ); 
}

int i2poll(){    // everything happens here.  Call this from loop.
static int state = 0;
static int data;
   
   switch( state ){    
      case 0:      // idle state or between characters
        if( i2in != i2out ){   // get next character
           data = i2buf[i2out++];
           i2out &= (I2BUFSIZE - 1 );
           
           if( data & ISTART ){   // start
              data &= 0xff;
              // set start condition
              I2C0_C1 = 0xB0;      // enabled, master, TX
              state = 1; 
           }
           else if( data & ISTOP ){  // stop
              // set stop condition
              I2C0_C1 = 0x90;         // clear master, enabled and tx set
              state = 3;
           }
           else{   // just data to send
              I2C0_D = data;
              state = 2;
           }
        }
      break; 
      case 1:  // wait for bus busy, send saved data which has the address
         if( (I2C0_S & 0x20) ){    // test busy bit
            state = 2;
            I2C0_D = data;
         }
      break;
      case 2:  // wait for transfer complete, blind to success or fail
         if( (I2C0_S & 0x80 ) ){  
            state = 0;
         }
      break;
      case 3:  // wait for stop, busy clear
         if( (I2C0_S & 0x20 ) == 0 ){
            state = 0;
         }
      break;    
   }
   
   if( i2in != i2out ) return (state + 8);
   else return state;
}
