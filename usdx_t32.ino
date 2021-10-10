/* 
 * uSDX with a Teensy 3.2
 * 
 * K1URC
 * 
 *   Hardware is a qrp-labs QCX+, starting with the original AM modulator connected to the BS170 drains.
 *   Current plan is to provide a provision to convert between the original uSDX processor and the Teensy
 *     by swapping a few parts in and out, with the Teensy mounted on the QRP-LABS development board.
 *   Will try using a Nokia display, daylight readable, fun to program, fast SPI operation.
 *   Alternate display:  OLED 128x64 I2C.   
 *     To compile both together need to have just one copy of current fonts as both 
 *     libraries are based on the same code from Rinky Dink Electronics.
 *     
 *     
 * Change log:     
 *    Version 1.0   Initial testing with a Weaver receiver.
 *    Version 1.2   Added provision for AGC and AM detector.  Renamed some audio objects.
 *    Version 1.21  New AM decoder.
 */

#define VERSION 1.21
 // QCX pin definitions  to Teensy 3.2
/* 
#define LCD_D4  0         //PD0    (pin 2)      N/C
#define LCD_D5  1         //PD1    (pin 3)      N/C
#define LCD_D6  2         //PD2    (pin 4)      N/C
#define LCD_D7  3         //PD3    (pin 5)      N/C
#define LCD_EN  4         //PD4    (pin 6)      N/C
#define FREQCNT 5         //PD5    (pin 11)     PULLUP ( @ 3.x k ) not needed, installed when debugging an issue
#define ROT_A   6         //PD6    (pin 12)     6
#define ROT_B   7         //PD7    (pin 13)     7
#define RX      8         //PB0    (pin 14)              + PullUp R36 installed
#define SIDETONE 9        //PB1    (pin 15)     A14  DAC wired seperately from SIDETONE net
#define KEY_OUT 10        //PB2    (pin 16)              + PullDown  10k ( need PWM pin )
#define SIG_OUT 11        //PB3    (pin 17)     PullDown  10k, no Teensy connection to SIG_OUT net
#define DAH     12        //PB4    (pin 18)
#define DIT     13        //PB5    (pin 19)
#define AUDIO1  14        //PC0/A0 (pin 23)     A2   ( or swap ? )
#define AUDIO2  15        //PC1/A1 (pin 24)     A3
#define DVM     16        //PC2/A2 (pin 25)
#define BUTTONS 17        //PC3/A3 (pin 26)     12     digital button active low
#define LCD_RS  18        //PC4    (pin 27)     N/C
#define SDA     18        //PC4    (pin 27)     A4
#define SCL     19        //PC5    (pin 28)     A5
//   Nokia Display
        RST                                      8
        CS                                      10
        D/C                                      9
        MOSI                                    11
        CLK                                     13   ?? does switching the LED generate noise, swap with pin 8?
*/

//  Pick a screen:  Nokia LCD or I2C 128x64 OLED
//  Running both together yields redefinition warnings as they are basically the same library, but it works.
#define USE_OLED
#define USE_LCD

#define DEBUG 1       // enables serial prints

// Nokia library uses soft SPI as the D/C pin needs to be held during data transfer.
// 84 x 48 pixels or 6 lines by 14 characters in text mode
#ifdef USE_LCD
  // I bypassed the pin to port code in this copy of the library as it didn't compile for ARM
  // Changed to use digitalWriteFast and renamed to _t version.
 #include <LCD5110_Basic_t.h>
#endif
#ifdef USE_OLED 
 #include <OLED1306_Basic.h>
#endif
#include <Wire.h>       // use wire library code to set up Pin Mux registers for I2C pins.
                        // all else using non-blocking routines here.

extern unsigned char SmallFont[];
extern unsigned char MediumNumbers[];
//extern unsigned char BigNumbers[];

#define ROW0   0          // text based rows
#define ROW1   8
#define ROW2  16
#define ROW3  24
#define ROW4  32
#define ROW5  40          // end of LCD
#define ROW6  48
#define ROW7  56          // OLED has two more text rows

#ifdef USE_LCD
   //   ( sclk mosi d/c rst cs )
   LCD5110 LCD( 13,11,9,8,10 );
#endif
#ifdef USE_OLED
   OLED1306 OLD;
#endif

#define EN_A  7            // encoder pins assignment
#define EN_B  6
#define EN_SW 12

/* button states */
#define IDLE_ 0
#define ARM 1
#define DTDELAY 2
#define DONE 3    
#define TAP  4
#define DTAP 5
#define LONGPRESS 6
#define DBOUNCE 50     // 60

// encoder users
#define FREQ 0
#define MENUS 1
#define VOLUME 2
int encoder_user;

// Menus:  Long press to enter
//   STICKY_MENU 0 :  Tap makes a selection and exits.   Double tap exits without a selection.
//   STICKY_MENU 1 :  Tap makes a selection.  Double tap makes a selection and exits.
#define STICKY_MENU 0

struct BAND_STACK{
   int mode;
   uint32_t freq;
   int stp;
   // could expand to relay commands, step, filter width etc.  Whatever plays nicely.
};

// "CW", "LSB", "USB", "AM", "Mem Tune", "WSPR"  mem tune special, think will remove wspr from this radio
#define CW 0
#define LSB 1
#define USB 2
#define AM 3 
struct BAND_STACK bandstack[] = {    // index is the band
  { LSB ,  3928000, 1000 },
  { AM  ,  6000000, 5000 },
  { LSB ,  7163000, 1000 },
  { CW  , 10105000,  100 },
  { USB , 14100000, 1000 },
  { USB  ,18101000, 1000 }
};

uint32_t freq = 18104600L;     // probably should init these vars from the bandstack
int step_ = 1000;
int band = 5;
int mode = 2;
int bfo;

int step_timer;                // allows double tap to backup the freq step to 500k , command times out
                               // and returns to the normal double tap command ( volume )
float af_gain = 0.3;
float rf_gain = 1.0;           // above 1 perhaps not a good idea
float sig_usb;                 // !!! maybe delete this temporary variable

/******************************** Teensy Audio Library **********************************/

// new AM detector

#include <Audio.h>
//#include <Wire.h>
//#include <SPI.h>
//#include <SD.h>
//#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=56.25,598.75
AudioSynthWaveformSine   Osc10k;          //xy=78.57144927978516,825.7143230438232
AudioAmplifier           amp1;           //xy=190,531.25
AudioAmplifier           amp2;           //xy=191.25,670
AudioEffectMultiply      FreqShift;      //xy=237.14286041259766,820.000002861023
AudioFilterBiquad        Q_filter;       //xy=346.8214111328125,669.0714111328125
AudioFilterBiquad        I_filter;       //xy=349.0357971191406,532.1072082519531
AudioAnalyzePeak         peak1;          //xy=394.2857142857142,397.1428571428571
AudioFilterBiquad        AM_HiPass;        //xy=402.8570861816406,820.0000286102295
AudioSynthWaveformSine   cosBFO;         //xy=419,608
AudioSynthWaveformSine   sinBFO;         //xy=419,757
AudioEffectMultiply      I_mixer;        //xy=507,537
AudioEffectMultiply      Q_mixer;        //xy=511,675
AudioEffectRectifier     Detector;       //xy=554.2857513427734,821.4285564422607
AudioAnalyzePeak         peak2;          //xy=598.5714073181152,398.5714359283447
AudioMixer4              Sub_SSB;      //xy=707,600
AudioFilterBiquad        AM_LoPass;        //xy=707.1427688598633,821.4285831451416
AudioMixer4              Mode_Select;         //xy=715.5357513427734,732.6785659790039
AudioAnalyzePeak         peak3;          //xy=811.4285163879395,401.4285125732422
AudioAnalyzeRMS          rms1;           //xy=866.25,516.75
AudioOutputAnalog        dac1;           //xy=885,710.1785888671875
AudioConnection          patchCord1(adcs1, 0, amp1, 0);
AudioConnection          patchCord2(adcs1, 0, peak1, 0);
AudioConnection          patchCord3(adcs1, 1, amp2, 0);
AudioConnection          patchCord4(Osc10k, 0, FreqShift, 1);
AudioConnection          patchCord5(amp1, I_filter);
AudioConnection          patchCord6(amp2, Q_filter);
AudioConnection          patchCord7(FreqShift, AM_HiPass);
AudioConnection          patchCord8(Q_filter, 0, Q_mixer, 0);
AudioConnection          patchCord9(I_filter, 0, I_mixer, 0);
AudioConnection          patchCord10(AM_HiPass, Detector);
AudioConnection          patchCord11(cosBFO, 0, I_mixer, 1);
AudioConnection          patchCord12(sinBFO, 0, Q_mixer, 1);
AudioConnection          patchCord13(I_mixer, 0, Sub_SSB, 1);
AudioConnection          patchCord14(I_mixer, peak2);
AudioConnection          patchCord15(Q_mixer, 0, Sub_SSB, 2);
AudioConnection          patchCord16(Detector, AM_LoPass);
AudioConnection          patchCord17(Sub_SSB, rms1);
AudioConnection          patchCord18(Sub_SSB, 0, Mode_Select, 0);
AudioConnection          patchCord19(Sub_SSB, peak3);
AudioConnection          patchCord20(Sub_SSB, 0, FreqShift, 0);
AudioConnection          patchCord21(AM_LoPass, 0, Mode_Select, 3);
AudioConnection          patchCord22(Mode_Select, dac1);
// GUItool: end automatically generated code

/*
// testing version

#include <Audio.h>
//#include <Wire.h>
//#include <SPI.h>
//#include <SD.h>
//#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=56.25,598.75
AudioAmplifier           amp1;           //xy=190,531.25
AudioAmplifier           amp2;           //xy=191.25,670
AudioFilterBiquad        Q_filter;       //xy=346.8214111328125,669.0714111328125
AudioFilterBiquad        I_filter;       //xy=349.0357971191406,532.1072082519531
AudioAnalyzePeak         peak1;          //xy=394.2857142857142,397.1428571428571
AudioSynthWaveformSine   cosBFO;         //xy=419,608
AudioSynthWaveformSine   sinBFO;         //xy=419,757
AudioEffectMultiply      I_mixer;        //xy=507,537
AudioEffectMultiply      Q_mixer;        //xy=511,675
AudioAnalyzePeak         peak2;          //xy=639.9999999999999,399.99999999999994
AudioMixer4              Add_SSB;         //xy=699.9999961853027,495.71427726745605
AudioMixer4              Sub_SSB;      //xy=707,600
AudioMixer4              Mode_Select;         //xy=715.5357513427734,732.6785659790039
AudioAnalyzeRMS          rms2;           //xy=863.75,462.5
AudioAnalyzeRMS          rms1;           //xy=866.25,516.75
AudioOutputAnalog        dac1;           //xy=885,710.1785888671875
AudioAnalyzePeak         peak3;          //xy=925.714241027832,598.5714015960693
AudioConnection          patchCord1(adcs1, 0, amp1, 0);
AudioConnection          patchCord2(adcs1, 0, peak1, 0);
AudioConnection          patchCord3(adcs1, 1, amp2, 0);
AudioConnection          patchCord4(amp1, I_filter);
AudioConnection          patchCord5(amp1, 0, Mode_Select, 3);
AudioConnection          patchCord6(amp2, Q_filter);
AudioConnection          patchCord7(Q_filter, 0, Q_mixer, 0);
AudioConnection          patchCord8(I_filter, 0, I_mixer, 0);
AudioConnection          patchCord9(cosBFO, 0, I_mixer, 1);
AudioConnection          patchCord10(sinBFO, 0, Q_mixer, 1);
AudioConnection          patchCord11(I_mixer, 0, Sub_SSB, 1);
AudioConnection          patchCord12(I_mixer, 0, Add_SSB, 1);
AudioConnection          patchCord13(I_mixer, peak2);
AudioConnection          patchCord14(Q_mixer, 0, Sub_SSB, 2);
AudioConnection          patchCord15(Q_mixer, 0, Add_SSB, 2);
AudioConnection          patchCord16(Add_SSB, rms2);
AudioConnection          patchCord17(Sub_SSB, rms1);
AudioConnection          patchCord18(Sub_SSB, 0, Mode_Select, 0);
AudioConnection          patchCord19(Sub_SSB, peak3);
AudioConnection          patchCord20(Mode_Select, dac1);
// GUItool: end automatically generated code
*/

// Weaver mode receiver version 2
/*
#include <Audio.h>
//#include <Wire.h>
//#include <SPI.h>
//#include <SD.h>
//#include <SerialFlash.h>
// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=56.25,598.75
AudioAmplifier           amp1;           //xy=190,531.25
AudioAmplifier           amp2;           //xy=191.25,670
AudioFilterBiquad        Q_filter;       //xy=346.8214111328125,669.0714111328125
AudioFilterBiquad        I_filter;       //xy=349.0357971191406,532.1072082519531
AudioSynthWaveformSine   cosBFO;         //xy=419,608
AudioSynthWaveformSine   sinBFO;         //xy=419,757
AudioEffectMultiply      I_mixer;        //xy=507,537
AudioEffectMultiply      Q_mixer;        //xy=511,675
AudioMixer4              Add_SSB;         //xy=699.9999961853027,495.71427726745605
AudioMixer4              Sub_SSB;      //xy=707,600
AudioMixer4              Mode_Select;         //xy=715.5357513427734,732.6785659790039
AudioAnalyzeRMS          rms2;           //xy=863.75,462.5
AudioAnalyzeRMS          rms1;           //xy=866.25,516.75
AudioOutputAnalog        dac1;           //xy=885,710.1785888671875
AudioConnection          patchCord1(adcs1, 0, amp1, 0);
AudioConnection          patchCord2(adcs1, 1, amp2, 0);
AudioConnection          patchCord3(amp1, I_filter);
AudioConnection          patchCord4(amp1, 0, Mode_Select, 3);
AudioConnection          patchCord5(amp2, Q_filter);
AudioConnection          patchCord6(Q_filter, 0, Q_mixer, 0);
AudioConnection          patchCord7(I_filter, 0, I_mixer, 0);
AudioConnection          patchCord8(cosBFO, 0, I_mixer, 1);
AudioConnection          patchCord9(sinBFO, 0, Q_mixer, 1);
AudioConnection          patchCord10(I_mixer, 0, Sub_SSB, 1);
AudioConnection          patchCord11(I_mixer, 0, Add_SSB, 1);
AudioConnection          patchCord12(Q_mixer, 0, Sub_SSB, 2);
AudioConnection          patchCord13(Q_mixer, 0, Add_SSB, 2);
AudioConnection          patchCord14(Add_SSB, rms2);
AudioConnection          patchCord15(Sub_SSB, rms1);
AudioConnection          patchCord16(Sub_SSB, 0, Mode_Select, 0);
AudioConnection          patchCord17(Mode_Select, dac1);
// GUItool: end automatically generated code
*/
/* version 1
// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=175,597.5
AudioFilterBiquad        I_filter;       //xy=246,528
AudioFilterBiquad        Q_filter;       //xy=247,668
AudioSynthWaveformSine   cosBFO;         //xy=419,608
AudioSynthWaveformSine   sinBFO;         //xy=419,757
AudioEffectMultiply      I_mixer;        //xy=507,537
AudioEffectMultiply      Q_mixer;        //xy=511,675
AudioMixer4              Add_Select;      //xy=707,600
//AudioAnalyzeRMS          rms1;           //xy=900,528
AudioOutputAnalog        dac1;           //xy=900,591.4286117553711
AudioConnection          patchCord1(adcs1, 0, I_filter, 0);
AudioConnection          patchCord2(adcs1, 1, Q_filter, 0);
AudioConnection          patchCord3(I_filter, 0, I_mixer, 0);
AudioConnection          patchCord4(Q_filter, 0, Q_mixer, 0);
AudioConnection          patchCord5(cosBFO, 0, I_mixer, 1);
AudioConnection          patchCord6(sinBFO, 0, Q_mixer, 1);
AudioConnection          patchCord7(I_mixer, 0, Add_Select, 1);
AudioConnection          patchCord8(Q_mixer, 0, Add_Select, 2);
//AudioConnection          patchCord9(Add_Select, rms1);
AudioConnection          patchCord10(Add_Select, dac1);
// GUItool: end automatically generated code

*/


/**************************************************************************/
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
  Wire.setClock(400000);     // I2C0_F  40 = 100k ,  25 = 400k
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
  //i2poll();   // !!! did this cause an error?
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
      case 1:  // wait for bus busy (start done), send saved data which has the address
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

/********  code from the uSDX project   *********/
//  Copyright 2019, 2020   Guido PE1NNZ <pe1nnz@amsat.org>
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
//  is furnished to do so, subject to the following conditions: The above copyright notice and this
//  permission notice shall be included in all copies or substantial portions of the Software.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
//  DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//  The Si5351 code
class SI5351 {
public:
  volatile int32_t _fout;
  volatile uint8_t _div;  // note: uint8_t asserts fout > 3.5MHz with R_DIV=1
  volatile uint16_t _msa128min512;
  volatile uint32_t _msb128;
  volatile uint8_t pll_regs[8];

  #define BB0(x) ((uint8_t)(x))           // Bash byte x of int32_t
  #define BB1(x) ((uint8_t)((x)>>8))
  #define BB2(x) ((uint8_t)((x)>>16))

  #define FAST __attribute__((optimize("Ofast")))

  #define F_XTAL 27004300            // Crystal freq in Hz, nominal frequency 27004300
  //#define F_XTAL 25004000          // Alternate SI clock
  //#define F_XTAL 20004000          // A shared-single 20MHz processor/pll clock
  volatile uint32_t fxtal = F_XTAL;

  inline void FAST freq_calc_fast(int16_t df)  // note: relies on cached variables: _msb128, _msa128min512, _div, _fout, fxtal
  { 
    #define _MSC  0x80000  //0x80000: 98% CPU load   0xFFFFF: 114% CPU load
    uint32_t msb128 = _msb128 + ((int64_t)(_div * (int32_t)df) * _MSC * 128) / fxtal;

    //#define _MSC  0xFFFFF  // Old algorithm 114% CPU load, shortcut for a fixed fxtal=27e6
    //register uint32_t xmsb = (_div * (_fout + (int32_t)df)) % fxtal;  // xmsb = msb * fxtal/(128 * _MSC);
    //uint32_t msb128 = xmsb * 5*(32/32) - (xmsb/32);  // msb128 = xmsb * 159/32, where 159/32 = 128 * 0xFFFFF / fxtal; fxtal=27e6

    //#define _MSC  (F_XTAL/128)  // 114% CPU load  perfect alignment
    //uint32_t msb128 = (_div * (_fout + (int32_t)df)) % fxtal;

    uint32_t msp1 = _msa128min512 + msb128 / _MSC;  // = 128 * _msa + msb128 / _MSC - 512;
    uint32_t msp2 = msb128 % _MSC;  // = msb128 - msb128/_MSC * _MSC;

    //pll_regs[0] = BB1(msc);  // 3 regs are constant
    //pll_regs[1] = BB0(msc);
    //pll_regs[2] = BB2(msp1);
    pll_regs[3] = BB1(msp1);
    pll_regs[4] = BB0(msp1);
    pll_regs[5] = ((_MSC&0xF0000)>>(16-4))|BB2(msp2); // top nibble MUST be same as top nibble of _MSC !
    pll_regs[6] = BB1(msp2);
    pll_regs[7] = BB0(msp2);
  }
  #define SI5351_ADDR 0x60              // I2C address of Si5351   (typical)

  inline void SendPLLBRegisterBulk(){
    i2start( SI5351_ADDR );         //i2c.start();
                                    //i2c.SendByte(SI5351_ADDR << 1);
    i2send( 26+1*8 + 3 );           //i2c.SendByte(26+1*8 + 3);  // Write to PLLB
    i2send( pll_regs[3]);           //i2c.SendByte(pll_regs[3]);
    i2send( pll_regs[4]);           //i2c.SendByte(pll_regs[4]);
    i2send( pll_regs[5]);           //i2c.SendByte(pll_regs[5]);
    i2send( pll_regs[6]);           //i2c.SendByte(pll_regs[6]);
    i2send( pll_regs[7]);           //i2c.SendByte(pll_regs[7]);
    i2stop();                       //i2c.stop();
  }

  void SendRegister(uint8_t reg, uint8_t* data, uint8_t n){
    i2start( SI5351_ADDR );         //    i2c.start();
                                    //i2c.SendByte(SI5351_ADDR << 1);
    i2send(reg);                    //i2c.SendByte(reg);
    while (n--) i2send(*data++);    //i2c.SendByte(*data++);
    i2stop();                       //i2c.stop();      
  }
  void SendRegister(uint8_t reg, uint8_t val){ SendRegister(reg, &val, 1); }
  int16_t iqmsa; // to detect a need for a PLL reset

  void freq(uint32_t fout, uint8_t i, uint8_t q){  // Set a CLK0,1 to fout Hz with phase i, q
      uint8_t msa; uint32_t msb, msc, msp1, msp2, msp3p2;
      uint8_t rdiv = 0;             // CLK pin sees fout/(2^rdiv)
      if(fout < 500000){ rdiv = 7; fout *= 128; }; // Divide by 128 for fout 4..500kHz

      uint16_t d = (16 * fxtal) / fout;  // Integer part
      if(fout > 30000000) d = (34 * fxtal) / fout; // when fvco is getting too low (400 MHz)

      if( (d * (fout - 5000) / fxtal) != (d * (fout + 5000) / fxtal) ) d++;
      // Test if multiplier remains same for freq deviation +/- 5kHz, if not use different divider
      // to make same
      if(d % 2) d++;  // even numbers preferred for divider (AN619 p.4 and p.6)
      uint32_t fvcoa = d * fout; 
        // Variable PLLA VCO frequency at integer multiple of fout at around 27MHz*16 = 432MHz
      msa = fvcoa / fxtal;     // Integer part of vco/fxtal
      msb = ((uint64_t)(fvcoa % fxtal)*_MSC) / fxtal; // Fractional part
      msc = _MSC;
      
      msp1 = 128*msa + 128*msb/msc - 512;
      msp2 = 128*msb - 128*msb/msc * msc;    // msp3 == msc        
      msp3p2 = (((msc & 0x0F0000) <<4) | msp2);  // msp3 on top nibble
      uint8_t pll_regs[8] = { BB1(msc), BB0(msc), BB2(msp1), BB1(msp1), BB0(msp1), BB2(msp3p2), BB1(msp2), BB0(msp2) };
      SendRegister(26+0*8, pll_regs, 8); // Write to PLLA
      SendRegister(26+1*8, pll_regs, 8); // Write to PLLB

      msa = fvcoa / fout;     // Integer part of vco/fout
      msp1 = (128*msa - 512) | (((uint32_t)rdiv)<<20);     // msp1 and msp2=0, msp3=1, not fractional
      uint8_t ms_regs[8] = {0, 1, BB2(msp1), BB1(msp1), BB0(msp1), 0, 0, 0};
      SendRegister(42+0*8, ms_regs, 8); // Write to MS0
      SendRegister(42+1*8, ms_regs, 8); // Write to MS1
      SendRegister(42+2*8, ms_regs, 8); // Write to MS2
      SendRegister(16+0, 0x0C|3|0x00);  // CLK0: 0x0C=PLLA local msynth; 3=8mA; 0x40=integer division; bit7:6=0->power-up
      SendRegister(16+1, 0x0C|3|0x00);  // CLK1: 0x0C=PLLA local msynth; 3=8mA; 0x40=integer division; bit7:6=0->power-up
      SendRegister(16+2, 0x2C|3|0x00);  // CLK2: 0x2C=PLLB local msynth; 3=8mA; 0x40=integer division; bit7:6=0->power-up
      SendRegister(165, i * msa / 90);  // CLK0: I-phase (on change -> Reset PLL)
      SendRegister(166, q * msa / 90);  // CLK1: Q-phase (on change -> Reset PLL)
      if(iqmsa != ((i-q)*msa/90)){ iqmsa = (i-q)*msa/90; SendRegister(177, 0xA0); } // 0x20 reset PLLA; 0x80 reset PLLB
      SendRegister(3, 0b11111100);      // Enable/disable clock

      _fout = fout;  // cache
      _div = d;
      _msa128min512 = fvcoa / fxtal * 128 - 512;
      _msb128=((uint64_t)(fvcoa % fxtal)*_MSC*128) / fxtal;
  }

/*
  uint8_t RecvRegister(uint8_t reg){
    i2c.start();  // Data write to set the register address
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(reg);
    i2c.stop();
    i2c.start(); // Data read to retrieve the data from the set address
    i2c.SendByte((SI5351_ADDR << 1) | 1);
    uint8_t data = i2c.RecvByte(true);
    i2c.stop();
    return data;
  }
*/  
  void powerDown(){
    for(int addr = 16; addr != 24; addr++) SendRegister(addr, 0b10000000);  // Conserve power when output is disabled
    SendRegister(3, 0b11111111); // Disable all CLK outputs    
  }
  #define SI_CLK_OE 3     // ??

};
static SI5351 si5351;


//***********************************************************

void setup() {
   int contrast = 68;

   if( DEBUG ) Serial.begin(9600);

   pinMode(EN_A,INPUT_PULLUP);
   pinMode(EN_B,INPUT_PULLUP);
   pinMode(EN_SW,INPUT_PULLUP);

   i2init();

   #ifdef USE_LCD
    LCD.InitLCD(contrast);
    LCD.setFont(SmallFont);
    LCD.print((char *)("Version "),0,ROW5);    // just some junk to see on the screen
    LCD.printNumF(VERSION,2,6*8,ROW5);
   #endif

   #ifdef USE_OLED
     OLD.InitLCD();
     OLD.clrScr();
     OLD.setFont(SmallFont);
     OLD.print((char *)("Version "),0,ROW7);
     OLD.printNumF(VERSION,2,6*8,ROW7);
   #endif

   freq_display();
   status_display();
   encoder();             // pick up current position
   
   fake_s();
   qsy(freq);             // start Si5351

// Audio Library setup
  AudioNoInterrupts();
  AudioMemory(20);
  
  Sub_SSB.gain(1,1.0);   // correct sideband
  Sub_SSB.gain(2,-1.0);

  set_af_gain();
  set_rf_gain(rf_gain);
  set_Weaver_bandwidth(3000);

  // AM mode
  Osc10k.frequency(10000);
  Osc10k.amplitude(0.9);
  AM_HiPass.setHighpass(0,11000, 0.51763809);
  AM_HiPass.setHighpass(1,11000,0.70710678);
  AM_HiPass.setHighpass(2,11000,1.9318517);
  AM_HiPass.setNotch(3,10000,3.0);      // Q should be ?
  AM_LoPass.setLowpass(0,4000,0.50979558);
  AM_LoPass.setLowpass(1,4000,0.60134489);
  AM_LoPass.setLowpass(2,4000,0.89997622);
  AM_LoPass.setLowpass(3,4000,2.5629154);
  
  AudioInterrupts();


}

void set_af_gain(){

  if( mode == AM ){
     Mode_Select.gain(0,0.0);
     Mode_Select.gain(3,2*af_gain);   // gain boost on AM
  }
  else{
     Mode_Select.gain(3,0.0);
     Mode_Select.gain(0,af_gain);
  }
//  Mode_Select.gain(0,af_gain);    // SSB
//  Mode_Select.gain(3,0);          // AM

}

void set_rf_gain(float g ){

  amp1.gain(g);
  amp2.gain(g);
}


void set_Weaver_bandwidth(int bandwidth){
  
  bfo = bandwidth/2;                     // weaver audio folding at 1/2 bandwidth
  I_filter.setLowpass(0,bfo,0.50979558);       // filters are set to 1/2 the desired audio bandwidth
  I_filter.setLowpass(1,bfo,0.60134489);       // with Butterworth Q's for 4 cascade
  I_filter.setLowpass(2,bfo,0.89997622);
  I_filter.setLowpass(3,bfo,2.5629154);

  Q_filter.setLowpass(0,bfo,0.50979558);
  Q_filter.setLowpass(1,bfo,0.60134489);
  Q_filter.setLowpass(2,bfo,0.89997622);
  Q_filter.setLowpass(3,bfo,2.5629154);

  AudioNoInterrupts();                     // need so cos and sin start with correct phase

    // complex BFO
  cosBFO.amplitude(0.9);                   // amplitude 1.0 causes distortion ?
  cosBFO.frequency(bfo);
  cosBFO.phase(90);                        // cosine 
  sinBFO.amplitude(0.9);
  sinBFO.frequency(bfo);
  sinBFO.phase(0);                         // sine

  AudioInterrupts();
  
}


void fake_s(){
int i;
  
  OLD.gotoRowCol(1,0);
 // LCD.gotoRowCol(0,35);
  OLD.putch('S');
  OLD.write(0);
  for( i = 0x80; i < 0xff; ){
     OLD.write(i,2);  // bars 
     OLD.write(0);    // with spaces
     i >>= 1;
     i |= 0x80;
  }  
}

void loop() {
static uint32_t tm;
static int t2;            // distribute some processing
int t;  

   i2poll();

   //  1 ms routines
   if( tm != millis()){ 
      tm = millis();
      if( step_timer ) --step_timer;       // dtap step up to 500k enable 
      
      t = encoder();
      if( t ){
         if( encoder_user == MENUS ) top_menu(t);
         if( encoder_user == FREQ ){
            qsy( freq + (t * step_ ));
            freq_display();
         }
         if( encoder_user == VOLUME ) volume_adjust(t);
      }

      t = button_state(0);
      if( t > DONE ) button_process(t);

      // agc
      if( rms1.available() ) t2 = 3;          // flag to process on a different time tick

      // testing
      report_peaks();
   }

   if( t2 ){
     --t2;
     if( t2 == 1 ){
        sig_usb = rms1.read();
        agc_process( sig_usb);
     }
    // if( t2 == 0 && mode == AM ) aft_process();
   }
   
}

void report_peaks(){
static int count;

  if( ++count < 1000 ) return;            // once a second
  if( encoder_user != FREQ ) return;
  count = 0;

  LCD.print((char *)"Adc ",0,ROW3);
  LCD.print((char *)"Mix ",0,ROW4);
  LCD.print((char *)"Add ",0,ROW5);
  LCD.printNumF( peak1.read(),2,4*6,ROW3);
  LCD.printNumF( peak2.read(),2,4*6,ROW4);
  LCD.printNumF( peak3.read(),2,4*6,ROW5);
}

/*
void aft_process(){      // signal capture in AM mode for syncrodyne detection
static int t;
static int aft;
int ch;

  return;    // doesn't work
  
   if( ++t < 100 ) return;    // avoid this process producing audio artifacts
   t = 0;   ch = 0;
   if( sig_usb > sig_lsb + 0.05 ) --aft, ch = 1;
   if( sig_lsb > sig_usb + 0.05 ) ++aft, ch = 1;
   aft = constrain( aft,-200,200 );

   if( ch ) qsy( freq + aft );
}
*/

void button_process( int t ){

    switch( t ){
      case TAP:
        if( encoder_user == MENUS ){
          if( top_menu(2) == 0 ){              // selection made
              if( STICKY_MENU ) top_menu(0);   // back to main menu
              else menu_cleanup();
          }
        }
        else if( encoder_user == FREQ ){
           step_ /= 10;
           if( step_ == 500 ) step_ = 1000;
           if( step_ == 1 ) step_ = 1000;
           step_timer = 3000;                  // 3 seconds to dtap up past 1000
           status_display();
        }
      break;
      case DTAP:                               // volume,  RIT in menu I think
        if( encoder_user == MENUS ){
           if( STICKY_MENU ) top_menu(2);     // make selection and then
           menu_cleanup();                    // escape from menus
        }
        if( encoder_user == FREQ && step_timer && step_ < 500000){
           step_ *= 10;
           if( step_ == 10000 ) step_ = 5000;
           status_display();
        }
        // else volume toggle
        else{
            if( encoder_user == FREQ ) encoder_user = VOLUME, volume_adjust(0);
            else{
              volume_adjust(2);
              encoder_user = FREQ, status_display();
            }
        }
      break;   
      case LONGPRESS: encoder_user = MENUS, top_menu(0);  break;
    }
    button_state(DONE);
}


void volume_adjust( int val ){

   if( val == 0  ){     // first entry, clear status line
   
      #ifdef USE_LCD
        LCD.print((char*)"Volume ",0,ROW0);
        LCD.clrRow(0,6*7);
      #endif
      #ifdef USE_OLED
        OLD.print((char*)"Volume ",0,ROW2);
        OLD.clrRow(2,6*7);
      #endif
   }
   if( val == 2 ){          // exit, clean up status line
      #ifdef USE_LCD
        LCD.clrRow(0);
      #endif
      #ifdef USE_OLED
        OLD.clrRow(2);
      #endif
      return;    
   }

   if( af_gain > 1.0 ) val *= 2;                // bigger steps, crude log pot simulation
   if( af_gain > 2.0 ) val *= 2;
   af_gain = af_gain + ((float) val * 0.05);
   af_gain = constrain(af_gain,0.0,3.0);
   set_af_gain();

   // 
   #ifdef USE_LCD
      LCD.printNumF(af_gain,2,6*7,ROW0);
   #endif
   #ifdef USE_OLED
      OLD.printNumF(af_gain,2,6*7,ROW2);   
   #endif
  
}
      
void qsy( uint32_t f ){
static int cw_offset = 700;     // !!! make global ?, add to menu if want to change on the fly
  
    freq = f;
    //    noInterrupts();
    if(mode == CW){
      si5351.freq(freq - cw_offset, 0, 90);  // RX in USB
      si5351.freq_calc_fast(cw_offset); si5351.SendPLLBRegisterBulk(); // TX at freq
    }
    else if(mode == LSB) si5351.freq(freq, 90, 0);  // RX in LSB
    else si5351.freq(freq, 0, 90);  // RX in USB, AM
    //interrupts();
    
    //freq_display();     // need to delay screen update until after menu_cleanup
    
}

void status_display(){
const char modes[] = "CW LSBUSBAM ";
char msg[4];
char msg2[9];
char buf[20];
    
    if( mode > 3 ) return;           //!!! how to handle memory tuning
    strncpy(msg,&modes[3*mode],3);
    msg[3] = 0;
    
    // strcpy(msg2,"STP ");
    // if( step_ > 100000 ) strcpy(msg2,"1M"); changed steps to 500k 50k 5k 1k 100 10, 5k handy for am broadcast
    if( step_ > 100 ){
       itoa(step_/1000,buf,10);
       strcpy(msg2,buf);
       strcat(msg2,"K");
    }
    else{
      itoa(step_,buf,10);
      strcpy(msg2,buf);
    }
    while( strlen(msg2) < 4 ) strcat(msg2," ");
    msg2[4] = 0;
    

    #ifdef USE_LCD
     LCD.setFont(SmallFont);
     LCD.print(msg,0,0);
     LCD.print(msg2,6*4,0);
    #endif

    #ifdef USE_OLED
     OLD.setFont(SmallFont);
     OLD.print(msg,0,ROW2);
     OLD.print(msg2,6*4,ROW2);
    #endif
  
}


void band_change( int to_band ){

  bandstack[band].freq = freq;
  bandstack[band].mode = mode;
  bandstack[band].stp  = step_;
  band = to_band;
  mode = bandstack[band].mode;
  step_ = bandstack[band].stp;
  freq = bandstack[band].freq;
  mode_change(mode);
//  qsy( bandstack[band].freq );
//  status_display();            delay until after screen clear
  
}

void mode_change( int to_mode ){

  mode = to_mode;
  qsy( freq );            // to get phasing correct
  set_af_gain();
  if( mode == AM ){
    // Mode_Select.gain(0,0.0);
    // Mode_Select.gain(3,af_gain);
    set_Weaver_bandwidth(5000);    
  }
  else{
    // Mode_Select.gain(3,0.0);
    // Mode_Select.gain(0,af_gain);
    set_Weaver_bandwidth(3000);
  }
}


void menu_cleanup(){

   encoder_user = FREQ;
   #ifdef USE_LCD
    LCD.clrScr();
   #endif
   #ifdef USE_OLED
    OLD.clrScr();
   #endif

   freq_display();
   status_display(); 
}

void freq_display(){
int rem;
  
   rem = freq % 1000;
   #ifdef USE_LCD
    LCD.setFont(MediumNumbers);
    LCD.printNumI(freq/1000,0,ROW1,5,'/');       // '/' is a leading space with altered font table
    LCD.setFont(SmallFont);
    LCD.printNumI(rem,62,ROW2,3,'0');
   #endif

   #ifdef USE_OLED
    OLD.setFont(MediumNumbers);
    OLD.printNumI(freq/1000,4*12,ROW0,5,'/');
    OLD.setFont(SmallFont);
    OLD.printNumI(rem,9*12,ROW1,3,'0');
   #endif
}


#define AGC_FLOOR  0.035             // was 0.07
#define AGC_SLOPE  0.8             // was .8
#define AGC_HANG   300             // ms
void agc_process( float reading ){
static float sig = AGC_FLOOR;
static int hang;
float g;
int ch;                            // flag change needed

    ch = 0;
    reading *= AGC_SLOPE;
    
    if( reading > sig && reading > AGC_FLOOR ){       // attack
       sig += 0.001,  hang = 0, ch = 1;
    }
    else if( sig > AGC_FLOOR && hang++ > AGC_HANG ){  // decay
       sig -= 0.00005, ch = 1;
    }

    if( ch ){                                         // change needed
            OLD.printNumF( sig,2,0,ROW6 );            // debug
       g = 1.0 - sig + AGC_FLOOR;
       set_rf_gain( g * rf_gain );                    // front end gain.  Only manual rf_gain can increase it.
    }                                                 // AGC can only decrease it.
                                                      // may have to adjust FLOOR with manual rf_gain changes.
}                                                     // see how this works first.


int encoder(){         /* read encoder, return 1, 0, or -1 */
static int mod;        /* encoder is divided by 4 because it has detents */
static int dir;        /* need same direction as last time, effective debounce */
static int last;       /* save the previous reading */
int new_;              /* this reading */
int b;

   new_ = (digitalReadFast(EN_B) << 1 ) | digitalReadFast(EN_A);
   if( new_ == last ) return 0;       /* no change */

   b = ( (last << 1) ^ new_ ) & 2;  /* direction 2 or 0 from xor of last shifted and new data */
   last = new_;
   if( b != dir ){
      dir = b;
      return 0;      /* require two in the same direction serves as debounce */
   }
   mod = (mod + 1) & 3;       /* divide by 4 for encoder with detents */
   if( mod != 0 ) return 0;

   return ( (dir == 2 ) ? 1: -1 );   /* swap defines EN_A, EN_B if it works backwards */
}


int button_state( int fini ){     /* state machine running at 1ms rate */
static int press_,nopress;
static int st;
int sw;

      if( fini ){                // switch state latched until processed and we say done
          st = DONE;
          return st;
      }
      sw = digitalReadFast(EN_SW) ^ 1;   
      if( sw ) ++press_, nopress= 0;
      else ++nopress, press_= 0;
      
      /* switch state machine */
         if( st == IDLE_ && press_ >= DBOUNCE ) st = ARM;
         if( st == DONE && nopress >= DBOUNCE ) st = IDLE_;       /* reset state */

         /* double tap detect */
         if( st == ARM && nopress >= DBOUNCE/2 )  st = DTDELAY;
         if( st == ARM && press_ >= 8*DBOUNCE )  st = LONGPRESS; 
         if( st == DTDELAY && nopress >= 4*DBOUNCE ) st = TAP;
         if( st == DTDELAY && press_ >= DBOUNCE )   st = DTAP;
          
     return st;        
}
/*****************   MENU's *******************************/
struct MENU {
  const int no_sel;                 // number of selections
  const char title[15];             // top line - 14 char max in title
  const char choice[8][9];          // selections to display - max 8 selections - max text length is 8
};

struct MENU mode_menu = {
   6,
   "Select Mode",
   { "CW", "LSB", "USB", "AM", "Mem Tune", "WSPR" }          
};


struct MENU band_menu = {
   6,
   "Amateur Band",
   {"80", "60", "40", "30", "20", "17" }
};

struct MENU main_menu = {
  2,
  "Top Menu",
  { "Band", "Mode" }
};


// pass value 0 - init, 1 or -1 encoder, 2 = selection made
// return 0 when done
// this function is called repeatedly during the selection process
int top_menu( int function ){
static int top_sel;    // where we are in the top most menu
static int state;      // where we are in the selection process
static struct  MENU *active_menu;  // menu pointer so can have common code for all menu's
static int def_val;    // current default value in selection process

int ret_val;

   ret_val = 1;   // assume we are still busy

   if( function == 0 ) {
     top_menu2( top_sel, &main_menu, 0 );   // init a new main menu
     active_menu = &main_menu;
     def_val = top_sel;
     state = 0;
   }
   else if( function == 1 || function == -1 ){  // encoder 
      def_val = top_menu2( def_val, active_menu, function );   // move through values with the encoder
   }
   else{   // the switch was toggled, pick a selection and act upon it
       switch (state) {
         case 0:   // top menu was shown,  pick a submenu to display
            top_sel = def_val;
            switch ( top_sel ){
              case 0: active_menu = &band_menu;  def_val = band; state = 1; break;
              case 1: active_menu = &mode_menu;  def_val = mode; state = 2; break;
              //case 2: active_menu = &multi_1_menu; def_val = multi1; state = 3; break;
              //case 3: active_menu = &multi_2_menu; def_val = multi2; state = 4; break;              
            }
            //if( transmitting && top_sel < 2 ) tx_off(1);
            def_val = top_menu2(def_val,active_menu, 0 );   // init a new submenu
         break;
         case 1:   // a new band was selected or not
            band_change( def_val );
            ret_val = state = 0;
         break;
         case 2:
            mode_change( def_val );
            ret_val = state = 0;
         break;   // mode change
       //  case 3:  multi1 = def_val; rit_onoff(0); ret_val = state = 0; break; 
       //  case 4:  multi2 = def_val; rit_onoff(0); ret_val = state = 0; break; 
         //default:  state = 0; ret_val = 0; break;  // temp
       }  // end switch
   }
   
   return ret_val;
}

// this function was changed to allow repeated calls
// move the highlighed selection through the menu
int top_menu2(int def_val, struct MENU *m, int encode_){     // return the menu value selected/highlighted

static int old_val;   // this is perhaps redundant as the top menu also keeps track of the current selection

  if( encode_ == 0 ){   // init a new menu
     old_val= def_val;
     #ifdef USE_OLED
       OLD.clrScr(); 
       OLD.setFont( SmallFont );
     #endif
     #ifdef USE_LCD  
       LCD.clrScr(); 
       LCD.setFont( SmallFont );
     #endif  
     show_menu(def_val,m);
     return def_val;
  }
 
  def_val = old_val;  

     def_val = def_val + encode_;     // turning the tuning knob to highlight a selection
     if( def_val < 0 ) def_val = 0;
     if( def_val >= m->no_sel ) def_val = m->no_sel -1;
     if( def_val != old_val){
       old_val= def_val;
       show_menu(def_val,m);            // show the new highlighted value 
     }    
   return def_val; 
}


// 5 choices display, LCD uses rows 1 to 5, OLED uses rows 2 to 6
void show_menu( int sel, struct MENU *m ){  // display menu on OLED display
int i;
static int base;

   while( sel > base + 4 ) ++base;    // move the menu window around only when needed
   while( sel < base ) --base;        // so the selected choice is in range
   if( m->no_sel <= 5 ) base = 0;     // most of the menu's will not need a base

   #ifdef USE_OLED
    OLD.print(m->title,0,ROW1);
   #endif
   #ifdef USE_LCD
    LCD.print(m->title,0,ROW0);
   #endif 
   
   for( i= 0; i < m->no_sel; ++i ){
      if( i < base ) continue;
      if( i > base + 4 ) break;
      #ifdef USE_OLED
       if( i == sel ) OLD.invertText( 1 );
       else OLD.invertText( 0 );
       OLD.clrRow( i - base + 2 );
       OLD.print(m->choice[i], 16, 8 * (i - base + 1) + ROW1 );
      #endif
      #ifdef USE_LCD
       if( i == sel ) LCD.invertText( 1 );
       else LCD.invertText( 0 );
       LCD.clrRow( i - base + 1 );
       LCD.print(m->choice[i], 16, 8 * (i - base + 1) );
      #endif
   }
   #ifdef USE_OLED
    OLD.invertText(0);
   #endif
   #ifdef USE_LCD
    LCD.invertText(0);
   #endif 
 
 // show some hint on the screen if there are more choices than what is displayed
   #ifdef USE_OLED
    if( base != 0 ) OLD.print((char * )("--"),RIGHT,ROW2); 
    if( (m->no_sel - base)  > 5 ) OLD.print((char * )("++"),RIGHT,ROW6);
   #endif
   #ifdef USE_LCD
    if( base != 0 ) LCD.print((char * )("--"),RIGHT,ROW1); 
    if( (m->no_sel - base)  > 5 ) LCD.print((char * )("++"),RIGHT,ROW5);
   #endif

}
