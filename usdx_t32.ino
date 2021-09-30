/* 
 * uSDX with a Teensy 3.2
 * 
 * K1URC
 * 
 *   Hardware is a qrp-labs QCX+, starting with the original AM modulator connected to the BS170 drains.
 *   Current plan is to provide a provision to convert between the original uSDX processor and the Teensy
 *     by swapping a few parts in and out, with the Teensy mounted on the QRP-LABS development board.
 *   Will try using a Nokia display, daylight readable, fun to program, fast SPI operation.
 *   Alternate display:  OLED 128x64 I2C.  Basically using the same library as the Nokia so can't compile 
 *   both together.
 *     
 *     
 */

// testing display with character based library
// this library uses soft SPI but I wired to the hardware SPI pins
// 84 x 48 pixels or 6 lines by 14 characters in text mode

// removed pin to port code from the library in this copy. Changed to digitalWriteFast.
#include <LCD5110_Basic_t.h>
//#include <OLED1306_Basic.h>
#include <Wire.h>                 // use wire code to set up Pin Mux registers

extern unsigned char SmallFont[];
extern unsigned char MediumNumbers[];
//extern unsigned char BigNumbers[];

#define ROW0   0          // text based rows
#define ROW1   8
#define ROW2  16
#define ROW3  24
#define ROW4  32
#define ROW5  40

//   ( sclk mosi d/c rst cs )
LCD5110 LCD( 13,11,9,8,10 );   //
//OLED1306 OLD;


uint32_t freq = 18104600L;


//***********************************************************

void setup() {
   int contrast = 68;
   LCD.InitLCD(contrast);
   LCD.setFont(SmallFont);

   //char msg[] = "Radio";
   //LCD.print(msg,0,ROW3);
   // some junk writes to screen
   LCD.print((char *)("Radio Active23"),0,ROW5); 
   LCD.print((char *)("USB"), 0,ROW0 );

   freq_display();
}

void loop() {

   i2poll();
   
}

void freq_display(){
int rem;
  
   LCD.setFont(MediumNumbers);
   LCD.printNumI(freq/1000,0,ROW1,5,'/');       // '/' is a leading space with altered font table
   LCD.setFont(SmallFont);
   rem = freq % 1000;
   LCD.printNumI(rem,62,ROW2,3,'0');   
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


void i2flush(){  //  call poll to empty out the buffer.  This one does block.

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
/************************************************************************/
