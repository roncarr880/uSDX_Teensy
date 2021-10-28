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
 * Change/Progress log:     
 *    Version 1.0   Initial testing with a Weaver receiver.
 *    Version 1.2   Added provision for AGC and AM detector.  Renamed some audio objects.
 *    Version 1.21  New AM decoder.
 *    Version 1.3   Baseband phasing version.  I think this version will have more reusable audio objects when transmitting.
 *    Version 1.31  Adding Mode Select object gets the volume control out of the AGC loop.
 *    Version 1.32  Moved AGC amps out of the phasing path as it degraded the audio image suppression.  The Weaver version also has
 *                  this problem.  Not sure I know what I am talking about here.  Figured it out, mixer default gain is 1, not zero.
 *    Version 1.33  Design changes with transmitting in mind.  Abandoning AM mode.              
 *    Version 1.34  Back to a Weaver decoder with FIR front end filters.  Then back to Bi-quads. 
 *    Version 1.35  Completed audio design.   USB objects commented out for now. Still need to cut VUSB etch on the Teensy 3.2
 *    Version 1.36  Changed I2C to a non-blocking DMA driven ( or ISR ) library i2c_t3.  My polling code commented out.  
 *                  This required changing the 7 audio library .ccp control files to use i2c_t3 instead of Wire to correct link errors.
 *                  A Teensyduino update will remove these changes ( duplicate wire definitions result ).  Also edited i2c_t3.h to
 *                  allow only 1 I2C bus to run.  This is a user option as noted in the file.
 *    Version 1.37  Added a Audio Library object for an AM decoder as an exercise in learning how to add a custom audio object.              
 *    Version 1.38  Added Hilberts for AM decoding, Tx magnitude and phase, and FFT analysis.  Some redundant elements in design now.
 *                  Removed the AM decoder Library object test.  Added AM detector at about 11k IF frequency.  Hilberts are multi
 *                  purpose. Can have only one of TX, FFT, AM running at any one time as each will need different filters.
 *    Version 1.39  Continuing with a Hilbert only version. Having issues with I and Q out of sync it seems.  No decode of SSB 
 *                  unless move some patchcords around or remove objects.  Rectify object was misbehaving. Updated my
 *                  Teensyduino install.
 *    Version 1.40  Wrote MagPhase object to extract data from the Audio library stream to use for transmitting.  Set _UA at             
 *                  44117/8 for initial testing.
 *    Version 1.41  I2C did not support sample rate 1/4 of 44k ( 11k ) for TX.  Changed MagPhase to use 1/6 rate or  7352. It does           
 *                  not divide evenly into 128 size buffer, but seems to be working ok.
 *    Version 1.42  Added another custom audio block ( SSB_AM ) to process receive signals at 1/4 rate with classic Hilbert. 
 *                  The upsample/filter seems to lose quite a bit of audio power.  Adjusted agc, and more volume is required.   
 *    
 */

 
#define VERSION 1.42

/*
 * The encoder switch works like this:
 * A tap changes the tuning step with 1k, 100, 10 in rotation.
 *   To change to a larger tuning step, first tap then double tap within 1.5 seconds and select 5k, 50k, 500k steps with double taps.
 * Normally a double tap brings up the volume control.  When done adjusting, double tap back to normal turning.
 * Long Press brings up the menu.  You can make selections with a tap.  Double tap exits.  And see sticky menus option below for 
 *   different behavior. 
 */

 // QCX pin definitions mega328 to Teensy 3.2
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

#include <i2c_t3.h>      // non-blocking wire library
#include "MagPhase.h"
#include "filters.h"
#include "SSB_AM.h"

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

// transmit interval timer
IntervalTimer EER_timer;

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
   int d;             // pre-calculated divider for each band. Can run Si5351 in spec except on 80 meters.
   // could expand to relay commands, step, filter width etc.  Whatever plays nicely.
};

// "CW", "LSB", "USB", "AM", "Mem Tune", "WSPR"  mem tune special, think will remove wspr from this radio
#define CW 0
#define LSB 1
#define USB 2
#define AM 3

struct BAND_STACK bandstack[] = {    // index is the band
  { LSB ,  3928000, 1000, 126 },
  { AM  ,  6000000, 5000, 126 },
  { LSB ,  7163000, 1000, 100 },
  { CW  , 10105000,  100,  68 },
  { USB , 14100000, 1000,  54 },
  { USB  ,18101000, 1000,  40 }
};

uint32_t freq = 18104600L;     // probably should init these vars from the bandstack
int step_ = 1000;
int band = 5;
int mode = 2;
int filter = 4;
// int bfo;
int magp;                      // !!! testing

int step_timer;                // allows double tap to backup the freq step to 500k , command times out
                               // and returns to the normal double tap command ( volume )
float af_gain = 0.3;
float agc_gain = 1.0;          // above 1 perhaps not a good idea.
float sig_rms;
int transmitting;

/******************************** Teensy Audio Library **********************************/ 

#include <Audio.h>
//#include <Wire.h>
//#include <SPI.h>
//#include <SD.h>
//#include <SerialFlash.h>


// 1/4 rate Hilbert followed by up sample by 4: custom blocks SSB_AM, MagPhase


// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=153.57142639160156,302.3809280395508
//AudioInputUSB            usb2;           //xy=253.28580474853516,458.23806285858154
AudioAnalyzePeak         peak1;          //xy=340.7142753601074,185.95239639282227
AudioAmplifier           agc1;           //xy=341.42857360839844,250.00000190734863
AudioAmplifier           agc2;           //xy=342.85715103149414,367.14287185668945
AudioMixer4              TxSelect;          //xy=472.857177734375,458.05950927734375
AudioFilterFIR           QLow;           //xy=481.00001525878906,366.6666703224182
AudioFilterFIR           ILow;           //xy=484.00001525878906,249.6666703224182
AudioSSB_AM2             SSB_AM;        //xy=664.0000152587891,304.6666703224182
AudioMagPhase1           MagPhase;         //xy=673.8095512390137,457.14279556274414
AudioSynthWaveformSine   SideTone;       //xy=808.3333740234375,353.7499694824219
AudioFilterBiquad        BandWidth;        //xy=825,298.75
AudioAnalyzeRMS          rms1;           //xy=994.0000152587891,225.6666703224182
AudioMixer4              Volume;         //xy=994.0000152587891,305.6666703224182
AudioOutputAnalog        dac1;           //xy=1132.000015258789,262.6666703224182
//AudioOutputUSB           usb1;           //xy=1137.000015258789,345.6666703224182
AudioConnection          patchCord1(adcs1, 0, peak1, 0);
AudioConnection          patchCord2(adcs1, 0, agc1, 0);
AudioConnection          patchCord3(adcs1, 1, agc2, 0);
//AudioConnection          patchCord4(usb2, 0, TxSelect, 1);
AudioConnection          patchCord5(agc1, ILow);
AudioConnection          patchCord6(agc2, QLow);
AudioConnection          patchCord7(TxSelect, 0, MagPhase, 0);
AudioConnection          patchCord8(QLow, 0, SSB_AM, 1);
AudioConnection          patchCord9(QLow, 0, TxSelect, 0);
AudioConnection          patchCord10(ILow, 0, SSB_AM, 0);
AudioConnection          patchCord11(SSB_AM, BandWidth);
AudioConnection          patchCord12(SideTone, 0, Volume, 3);
AudioConnection          patchCord13(SideTone, 0, TxSelect, 3);
AudioConnection          patchCord14(BandWidth, 0, Volume, 0);
AudioConnection          patchCord15(BandWidth, rms1);
AudioConnection          patchCord16(Volume, dac1);
//AudioConnection          patchCord17(Volume, 0, usb1, 0);
//AudioConnection          patchCord18(Volume, 0, usb1, 1);
// GUItool: end automatically generated code



/*
// Hilbert version 

// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=153.57142639160156,302.3809280395508
//AudioInputUSB            usb2;           //xy=154.71432495117188,392.5237922668457
AudioSynthWaveformSine   TxTest;          //xy=158.5714225769043,458.5714054107666
AudioMixer4              TxSelect;          //xy=337.85715103149414,456.8095226287842
AudioAnalyzePeak         peak1;          //xy=340.7142753601074,185.95239639282227
AudioAmplifier           agc1;           //xy=341.42857360839844,250.00000190734863
AudioAmplifier           agc2;           //xy=342.85715103149414,367.14287185668945
AudioFilterFIR           Qm00;           //xy=481.00001525878906,366.6666703224182
AudioFilterFIR           Ip90;           //xy=484.00001525878906,249.6666703224182
AudioFilterBiquad        TxLow;        //xy=491.42856216430664,458.5714359283447
AudioMagPhase1           MagPhase;         //xy=647.1428718566895,477.1428699493408
AudioMixer4              Add_SSB;        //xy=664.0000152587891,304.6666703224182
AudioEffectRectifier     AMdet;          //xy=665.0000152587891,375.6666703224182
AudioFilterFIR           AMlow;          //xy=792.0000152587891,373.6666703224182
AudioFilterBiquad        BandWidth;      //xy=838.0000152587891,288.6666703224182
AudioSynthWaveformSine   SideTone;       //xy=880.0000152587891,426.6666703224182
AudioAnalyzeRMS          rms1;           //xy=994.0000152587891,225.6666703224182
AudioMixer4              Volume;         //xy=994.0000152587891,305.6666703224182
AudioOutputAnalog        dac1;           //xy=1132.000015258789,262.6666703224182
//AudioOutputUSB           usb1;           //xy=1137.000015258789,345.6666703224182
AudioConnection          patchCord1(adcs1, 0, peak1, 0);
AudioConnection          patchCord2(adcs1, 0, agc1, 0);
AudioConnection          patchCord3(adcs1, 1, agc2, 0);
AudioConnection          patchCord4(adcs1, 1, TxSelect, 0);
//AudioConnection          patchCord5(usb2, 0, TxSelect, 1);
AudioConnection          patchCord6(TxTest, 0, TxSelect, 3);
AudioConnection          patchCord7(TxSelect, TxLow);
//AudioConnection          patchCord7( TxSelect,0, MagPhase,0 );
AudioConnection          patchCord8(agc1, Ip90);
AudioConnection          patchCord9(agc2, Qm00);
AudioConnection          patchCord10(Qm00, 0, Add_SSB, 2);
AudioConnection          patchCord11(Ip90, 0, Add_SSB, 1);
AudioConnection          patchCord12(TxLow, 0, MagPhase, 0);
AudioConnection          patchCord13(Add_SSB, AMdet);
AudioConnection          patchCord14(Add_SSB, BandWidth);
AudioConnection          patchCord15(AMdet, AMlow);
AudioConnection          patchCord16(AMlow, 0, Volume, 1);
AudioConnection          patchCord17(BandWidth, 0, Volume, 0);
AudioConnection          patchCord18(BandWidth, rms1);
AudioConnection          patchCord19(SideTone, 0, Volume, 2);
AudioConnection          patchCord20(Volume, dac1);
//AudioConnection          patchCord21(Volume, 0, usb1, 0);
//AudioConnection          patchCord22(Volume, 0, usb1, 1);
// GUItool: end automatically generated code

*/
/*

// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=85.71429443359375,312.71428298950195
AudioInputUSB            usb2;           //xy=89.71429443359375,375.71428298950195
AudioAnalyzePeak         peak1;          //xy=255.71429443359375,157.71428298950195
AudioMixer4              RxTx2;          //xy=273,353.42859268188477
AudioMixer4              RxTx1;          //xy=275.71429443359375,235.71428298950195
AudioFilterFIR           Qm00;           //xy=411.142879486084,352.1428680419922
AudioFilterFIR           Ip90;           //xy=414.1429252624512,235.2857151031494
AudioMixer4              Sub_SSB;        //xy=594.1429061889648,290.85711097717285
AudioEffectRectifier     AMdet;          //xy=595.8571014404297,361.0000190734863
AudioMagPhase2           MagPhase;       //xy=622.5714797973633,192.28576278686523
AudioFilterFIR           AMlow;          //xy=722.9999847412109,359.5714340209961
AudioFilterBiquad        BandWidth;        //xy=768.5713768005371,274.2857131958008
AudioSynthWaveformSine   SideTone;         //xy=810.4286575317383,412.71426010131836
AudioAnalyzeRMS          rms1;           //xy=924.2859268188477,211.42855072021484
AudioMixer4              Volume;         //xy=924.2858772277832,291.4285659790039
AudioOutputAnalog        dac1;           //xy=1062.0001792907715,248.57140922546387
AudioOutputUSB           usb1;           //xy=1067.428768157959,331.7143135070801
AudioConnection          patchCord1(adcs1, 0, RxTx1, 0);
AudioConnection          patchCord2(adcs1, 0, RxTx1, 1);
AudioConnection          patchCord3(adcs1, 0, RxTx2, 1);
AudioConnection          patchCord4(adcs1, 0, peak1, 0);
AudioConnection          patchCord5(adcs1, 1, RxTx2, 0);
AudioConnection          patchCord6(usb2, 0, RxTx1, 2);
AudioConnection          patchCord7(usb2, 0, RxTx2, 2);
AudioConnection          patchCord8(RxTx2, Qm00);
AudioConnection          patchCord9(RxTx1, Ip90);
AudioConnection          patchCord10(Qm00, 0, MagPhase, 1);
AudioConnection          patchCord11(Qm00, 0, Sub_SSB, 2);
AudioConnection          patchCord12(Ip90, 0, MagPhase, 0);
AudioConnection          patchCord13(Ip90, 0, Sub_SSB, 1);
AudioConnection          patchCord14(Sub_SSB, AMdet);
AudioConnection          patchCord15(Sub_SSB, BandWidth);
AudioConnection          patchCord16(AMdet, AMlow);
AudioConnection          patchCord17(AMlow, 0, Volume, 1);
AudioConnection          patchCord18(BandWidth, 0, Volume, 0);
AudioConnection          patchCord19(BandWidth, rms1);
AudioConnection          patchCord20(SideTone, 0, Volume, 2);
AudioConnection          patchCord21(Volume, dac1);
AudioConnection          patchCord22(Volume, 0, usb1, 0);
AudioConnection          patchCord23(Volume, 0, usb1, 1);
// GUItool: end automatically generated code
*/

// I2C functions that the OLED library expects to use.
void i2init(){

  Wire.begin(I2C_OP_MODE_DMA);   // use mode DMA or ISR 
  Wire.setClock(800000);     // I2C0_F  40 = 100k ,  25 = 400k.  800000 seems to work, returns 818, 600k returns 600
                             // clock stretching may produce reduced speed if clock is way to fast for the devices.
                             // Calc that 700k speed could maybe support a tx bandwidth of 5.5k at 11029 sample rate.
                             // It doesn't.  Nor does 800k or 1000k.
                             // At 1/2 rate of 1/4 rate:  500k works.  Using 700k for some margin of error.  1/8 rate overall.
                             // At 1/6 rate get some errors at 700k. Use 800k. Should have better TX quality at 1/6 rate.
                             // At 1/5 rate get some errors at 1000k.  Think we should stay at 1/6 rate. ( 1/6 of 44117 )
}

void i2start( unsigned char adr ){

  while( Wire.done() == 0 );         // still busy with last transmission.  Need to block while still busy.
  Wire.beginTransmission( adr );

}

void i2send( unsigned int data ){ 

  Wire.write( data );
}

void i2stop( ){
  Wire.sendTransmission();     // non-blocking
 // Wire.endTransmission();      // blocking
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

  #define F_XTAL 27003380            // Crystal freq in Hz, nominal frequency 27004300
  //#define F_XTAL 25004000          // Alternate SI clock
  //#define F_XTAL 20004000          // A shared-single 20MHz processor/pll clock
  volatile uint32_t fxtal = F_XTAL;

  inline void FAST freq_calc_fast(int16_t df)  // note: relies on cached variables: _msb128, _msa128min512, _div, _fout, fxtal
  { 
// static int mod;   // !!! test
    #define _MSC  0x80000  //0x80000: 98% CPU load   0xFFFFF: 114% CPU load
    uint32_t msb128 = _msb128 + ((int64_t)(_div * (int32_t)df) * _MSC * 128) / fxtal;

    //#define _MSC  0xFFFFF  // Old algorithm 114% CPU load, shortcut for a fixed fxtal=27e6
    //register uint32_t xmsb = (_div * (_fout + (int32_t)df)) % fxtal;  // xmsb = msb * fxtal/(128 * _MSC);
    //uint32_t msb128 = xmsb * 5*(32/32) - (xmsb/32);  // msb128 = xmsb * 159/32, where 159/32 = 128 * 0xFFFFF / fxtal; fxtal=27e6

// ++mod;
// if( mod > 0 && mod < 50 ){
  //  Serial.print( df );  Serial.write(' ');     // !!! testing get a burst
  //  Serial.println( magp ); 
// }
// if( mod > 20000 ) mod = 0;

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
      
      //if(fout < 500000){ rdiv = 7; fout *= 128; }; // Divide by 128 for fout 4..500kHz
      //uint16_t d = (16 * fxtal) / fout;  // Integer part
      //if(fout > 30000000) d = (34 * fxtal) / fout; // when fvco is getting too low (400 MHz)
      
      uint16_t d = bandstack[band].d;
      if( (d * (fout - 5000) / fxtal) != (d * (fout + 5000) / fxtal) ) d -= 2;     // d++;
      // Test if multiplier remains same for freq deviation +/- 5kHz, if not use different divider to make same
      // if(d % 2) d++; // forced even divider from bandstack // even numbers preferred for divider (AN619 p.4 and p.6)
      uint32_t fvcoa = d * fout; 
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

  //Serial.print( fout/1000 ); Serial.write(' ');
  //Serial.print( fvcoa/1000 ); Serial.write(' ');
  //Serial.print( iqmsa );  Serial.write(' ');
  //Serial.println(d);

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
//  void powerDown(){
//    for(int addr = 16; addr != 24; addr++) SendRegister(addr, 0b10000000);  // Conserve power when output is disabled
//    SendRegister(3, 0b11111111); // Disable all CLK outputs    
//  }
//  #define SI_CLK_OE 3 

};
static SI5351 si5351;


//***********************************************************


// the transmit process uses I2C in an interrupt context.  Must prevent other users from writing.  
// No frequency changes or any OLED writes.  transmitting variable is used to disable large parts of the system.
#define DRATE 6                      // decimation rate used in MagPhase
#define F_SAMP_TX (44117/DRATE)
//#define _UA (44117/(2*DRATE))          // match definition in MagPhase.cpp
#define _UA 8448
#define R_SAMP_UA 0.870363794          // float of F_SAMP_TX/_UA   (44117/6) / 8448

int eer_count;
int temp_count;          // !!! debug
int eer_adj;             // !!! debug
int overs;               // !!! debug
// float eer_time = 90.680;  //90.668;  // us for each sample deci rate 4
float eer_time = 136.0;  // 1/6 rate ( 1/6 of 44117 )
// float eer_time = 113.335;   // 1/5 rate

void EER_function(){     // EER transmit interrupt function.  Interval timer.
int c;
static int prev_phase;
int phase_;
int mag;

   if( MagPhase.available() == 0 ){       // start with 6 ms of buffered data( more now with 1/6 sample rate )
      eer_count = 0;
      return;
   }

   // process Mag and Phase
   phase_ = MagPhase.pvalue(eer_count);
   int dp = phase_ - prev_phase;
   prev_phase = phase_;

      if( dp < 0 ) dp = dp + _UA;

      //dp *= 2;    // (F_SAMP_TX / _UA);
      dp = (float)dp * R_SAMP_UA;
      if( dp < 3100 /*F_SAMP_TX / 2 */ ){               // skip sending out of tx bandwidth freq
         if( mode == LSB ) dp = -dp;
         si5351.freq_calc_fast(dp);
         if( Wire.done() )                    // crash all:  can't wait for I2C while in ISR
           si5351.SendPLLBRegisterBulk();
           else ++overs;                   // !!! debug only : count missed transmissions
      }     
           
   mag = MagPhase.mvalue(eer_count);
   // will start with 10 bits, scale up or down, test if over 1024, write PWM pin for KEY_OUT.
   magp = mag >> 5;

   ++eer_count;
   eer_count &= ( AUDIO_BLOCK_SAMPLES - 1 );

   // adjust timing.  MagPhase data counter reported will be 0, 32, 64, 96.
   // we want to be reading out the data two blocks behind ( we decimated by 4, these blocks are 32 in length )
   // testing at eer_count == zero, this test happens once per 12ms. 
   // that is how it used to work.  Now decimating by 6 and each 3ms block of 128 results in 21 or 22 samples.
   // So we are 3 blocks behind but that doesn't really matter for this sync routine.  We should still hit an index of 64 
   // in the middle of the buffered data.  I2C couldn't keep up with decimation rates of 4 or 5, so using 6.
   int u = 0;
   if( eer_count == 0 ){
      c = MagPhase.read_count();
      temp_count = c;               // !!! debug
      //if( c == 64 ) ;                                    // two blocks delay is the goal
      if( c <  64-8 ) eer_time += 0.0001, ++eer_adj, ++u;    // slow down  64-8 for 1/4 1/6 versions.  76-8 for 1/5 rate.
      if( c > 64+8 ) eer_time -= 0.0001, --eer_adj, ++u;     // speed up    64-8 rates 1/4 and 1/6
     // leak timer toward what we think is correct
      if( eer_time > 136.01 ) eer_time -= 0.00001, ++u;   //  rate 1/6 version.    version at 1/4 rate use 90.67 to 90.68.
      if( eer_time < 135.99 ) eer_time += 0.00001, ++u;
     // if( eer_time > 113.34 ) eer_time -= 0.00001, ++u;     // rate 1/5 version
     // if( eer_time < 113.33 ) eer_time += 0.00001, ++u;     // use 76 +-8 as index to sync
      
      if( u ) EER_timer.update( eer_time);
   }
}

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
     OLD.printNumI( Wire.getClock()/1000,0,ROW4);
   #endif

   freq_display();
   status_display();
   encoder();             // pick up current position
   
  fake_s();

// Audio Library setup
  AudioNoInterrupts();
  AudioMemory(40);
 // AudioMemory(80);       // no help with no-decode issue
  
 // Add_SSB.gain(1,1.0);   // correct sideband
 // Add_SSB.gain(2,1.0);
 // Add_SSB.gain(0,0.0);
 // Add_SSB.gain(3,0.0);

//  RxTx1.gain(0,1.0);     // audio mux's
//  RxTx2.gain(0,1.0);     // Rx = 0, Tx mic = 1, Tx usb = 2
//  RxTx1.gain(1,0.0);
//  RxTx1.gain(2,0.0);
//  RxTx1.gain(3,0.0);
//  RxTx2.gain(1,0.0);
//  RxTx2.gain(2,0.0);
//  RxTx2.gain(3,0.0);

  TxSelect.gain(0,0.0);
  TxSelect.gain(1,0.0);
  TxSelect.gain(2,0.0);
 // TxSelect.gain(3,1.0);        // !!! testing
  TxSelect.gain(3,0.0);          // not testing
         
 // TxTest.amplitude( 0.8 );     // !!! testing
 // TxTest.frequency( 1800 );    // !!! testing

 // TxLow.setHighpass(0,300,0.70710678);
 // TxLow.setLowpass(1,2900,0.51763809);
 // TxLow.setLowpass(2,2900,0.70710678);
 // TxLow.setLowpass(3,2900,1.9318517);
  
  //agc1.gain(1.0);   redundant
  //agc2.gain(1.0);

  // 0.54119610   butterworth Q's two cascade
  // 1.3065630
  //  0.51763809  butterworth Q's 3 cascade
  //  0.70710678
  //  1.9318517

  set_af_gain(af_gain);
  set_agc_gain(agc_gain);
  
  filter = 4;
  set_bandwidth();
 // Ip90.begin(SSBp90,64);
 // Qm00.begin(SSBm00,64);

  ILow.begin(IQlow,32);
  QLow.begin(IQlow,32);
  
  qsy(freq);

  AudioInterrupts();

}


void set_bandwidth( ){
int bw;                              // or bandwidth changed in menu's.

  //  0.51763809  butterworth Q's 3 cascade
  //  0.70710678
  //  1.9318517


   if( mode == AM ){     // one highpass to restore dc level, lowpass wide
     BandWidth.setHighpass(0,300,0.70710678);
     BandWidth.setLowpass(1,3600,0.51763809);    
     BandWidth.setLowpass(2,3600,0.70710678);
     BandWidth.setLowpass(3,3600,1.9318517);    
     return;
   }

//   if( mode == AM ){     // set bandwidth to pass AM IF freq range for the AGC to work
//     BandWidth.setLowpass(0,11000,0.54119610);
//     BandWidth.setLowpass(1,11000,1.3065630);       // Butterworth Q's for 2 cascade
//     BandWidth.setHighpass(2,7000,0.54119610);
//     BandWidth.setHighpass(3,7000,1.3065630);    
//     return;
//   }

   bw = 3300;                        // remove warning uninitialized variable
   switch( filter ){
       case 0: bw = 300; break;     // data duplicated from menu strings.  Could be improved so that
       case 1: bw = 700; break;     // one doesn't need to maintain data in two places if changes are made.
       case 2: bw = 2700; break;
       case 3: bw = 3000; break;
       case 4: bw = 3300; break;
       case 5: bw = 3600; break;
   }

  if( bw > 1200 ){
     BandWidth.setLowpass(0,bw,0.50979558);
     BandWidth.setLowpass(1,bw,0.60134489);       // Butterworth Q's for 4 cascade
     BandWidth.setLowpass(2,bw,0.89997622);
     BandWidth.setLowpass(3,bw,2.5629154);
  }
  // if bandwidth is below a certain value, assume CW and split
  // two stages of lowpass and two of highpass.   Q would be
  // 0.54119610
  // 1.3065630
  else{
     BandWidth.setLowpass(0,700+bw/2,0.54119610);
     BandWidth.setLowpass(1,700+bw/2,1.3065630);       // Butterworth Q's for 2 cascade
     BandWidth.setHighpass(2,700-bw/2,0.54119610);
     BandWidth.setHighpass(3,700-bw/2,1.3065630);    
  }
  
}

void set_af_gain(float g){

   //Volume.gain(g);

/*
  if( mode == AM ){
     Volume.gain(0,0.0);
     Volume.gain(1,g);
  }
  else{
     Volume.gain(1,0.0);
     Volume.gain(0,g);    
  }
  */
  Volume.gain(0,g);
  Volume.gain(1,0.0);
  Volume.gain(2,0.0);     // unused
  Volume.gain(3,0.0);     // sidetone
}

void set_agc_gain(float g ){

  AudioNoInterrupts();
 //   RxTx1.gain(0,g);     // audio mux's
 //   RxTx2.gain(0,g);     // Rx = 0, Tx mic = 1, Tx usb = 2
    agc1.gain(g);
    agc2.gain(g);
  AudioInterrupts();
}


/*
void set_Weaver_bandwidth(int bandwidth){
  
  bfo = bandwidth/2;                           // weaver audio folding at 1/2 bandwidth
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
  qsy(freq);             // refresh Si5351 to new vfo frequency after weaver bandwidth changes
                      
}
*/


/********************
void set_tx_bandwidth(){   // some duplicate code to just mess with the filters and not the BFO
                           // the tx filters need to pass the full bandwidth and not just half as in RX mode.

  I_filter.setLowpass(0,2700,0.50979558);       // Fixed 3db down at 2700 for TX
  I_filter.setLowpass(1,2700,0.60134489);       // Butterworth Q's for 4 cascade
  I_filter.setLowpass(2,2700,0.89997622);
  I_filter.setLowpass(3,2700,2.5629154);

  Q_filter.setLowpass(0,2700,0.50979558);
  Q_filter.setLowpass(1,2700,0.60134489);
  Q_filter.setLowpass(2,2700,0.89997622);
  Q_filter.setLowpass(3,2700,2.5629154);  
}

void set_rx_bandwidth(){             // restore receive bandwidth after TX
int bw;                              // or bandwidth changed in menu's.

   bw = 3300;                        // remove warning uninitialized variable
   switch( filter ){
       case 0: bw = 2200; break;     // data duplicated from menu strings.  Could be improved so that
       case 1: bw = 2200; break;     // one doesn't need to maintain data in two places if changes are made.
       case 2: bw = 2700; break;
       case 3: bw = 3000; break;
       case 4: bw = 3300; break;
       case 5: bw = 3600; break;
   }

  //if( mode == AM ) bw = 2*4500;   // 4500 for AM, weaver not used but for AGC  !!! needed anymore ?
  bfo = bw/2;                                  // weaver audio folding at 1/2 bandwidth

  if( filter > 1 ){                            // ssb filters
     I_filter.setLowpass(0,bfo,0.50979558);       // filters are set to 1/2 the desired audio bandwidth
     I_filter.setLowpass(1,bfo,0.60134489);       // with Butterworth Q's for 4 cascade
     I_filter.setLowpass(2,bfo,0.89997622);
     I_filter.setLowpass(3,bfo,2.5629154);

     Q_filter.setLowpass(0,bfo,0.50979558);
     Q_filter.setLowpass(1,bfo,0.60134489);
     Q_filter.setLowpass(2,bfo,0.89997622);
     Q_filter.setLowpass(3,bfo,2.5629154);

     set_af_gain(af_gain);
  }
  else{                                           // CW filters
     // 0.54119610   butterworth Q's two cascade
     // 1.3065630

     I_filter.setLowpass(0,600+100*filter,0.54119610);       // filter at 1/2 of desired cw tone
     I_filter.setLowpass(1,600+100*filter,1.3065630); 
     I_filter.setHighpass(2,250-100*filter,0.54119610);        
     I_filter.setHighpass(3,250-100*filter,1.3065630); 
                                               
     Q_filter.setLowpass(0,600+100*filter,0.54119610);
     Q_filter.setLowpass(1,600+100*filter,1.3065630);
     Q_filter.setHighpass(2,250-100*filter,0.54119610);
     Q_filter.setHighpass(3,250-100*filter,1.3065630); 
     
    
    
     I_filter.setLowpass(0,bfo,0.51763809);       // filters are set to a value without distortion
     I_filter.setLowpass(1,bfo,0.70710678);       // with Butterworth Q's for 3 cascade
     I_filter.setLowpass(2,bfo,1.9318517);        
     I_filter.setBandpass(3,750/2,(filter == 0) ? 7:3);       // and one stage of bandpass at 1/2 audio tone?
                                                              // is this correct? Can this be done with Weaver RX?
                                                              // volume is greatly reduced
     Q_filter.setLowpass(0,bfo,0.51763809);
     Q_filter.setLowpass(1,bfo,0.70710678);
     Q_filter.setLowpass(2,bfo,1.9318517);
     Q_filter.setBandpass(3,750/2,(filter == 0) ? 7:3); 

     set_af_gain( 3 * af_gain );
     
  }
  
  AudioNoInterrupts();                     // need so cos and sin start with correct phase

    // complex BFO
  cosBFO.amplitude(0.9);                   // amplitude 1.0 causes distortion ?
  cosBFO.frequency(bfo);
  cosBFO.phase(90);                        // cosine 
  sinBFO.amplitude(0.9);
  sinBFO.frequency(bfo);
  sinBFO.phase(0);                         // sine

  AudioInterrupts();
  qsy(freq);             // refresh Si5351 to new vfo frequency after weaver bandwidth changes  
}
*/

void tx(){
  // what needs to change to enter tx mode
  // audio mux switching, tx bandwidth, pwm, SI5351 clocks on/off, I2C speed change, ...
  transmitting = 1;
  si5351.SendRegister(3, 0b11111011);      // Enable clock 2
  MagPhase.setmode(1);
  EER_timer.begin(EER_function,eer_time);
  
}

void rx(){
  // what needs to change to return to rx mode.
  EER_timer.end();
  MagPhase.setmode(0);
  transmitting = 0;
  si5351.SendRegister(3, 0b11111100);      // Enable clock 0 and 1

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
int t;  


   t = encoder();
   if( t ){
      if( encoder_user == MENUS ) top_menu(t);
      if( encoder_user == FREQ ){
         qsy( freq + (t * step_ ));
         freq_display();
      }
      if( encoder_user == VOLUME ) volume_adjust(t);
   }

      // agc
   if( rms1.available() ){  
        sig_rms = rms1.read();
        agc_process( sig_rms);
        report_peaks();
   }


   //  1 ms routines
   if( tm != millis()){ 
      tm = millis();
      if( step_timer ) --step_timer;       // 1.5 seconds to dtap freq step up to 500k 
      
      t = button_state(0);
      if( t > DONE ) button_process(t);

   }
   
}

void report_peaks(){
static int count;              // !!! think about PWM'ing the RX pin for part of AGC process

  if( ++count < 1000 ) return;            // once a second
  if( encoder_user != FREQ ) return;
  count = 0;

 // eer_test();                       // !!! testing
/*
  LCD.print((char *)"Adc ",0,ROW3);
  LCD.print((char *)"H45 ",0,ROW4);
  LCD.print((char *)"Add ",0,ROW5);
  LCD.printNumF( peak1.read(),2,4*6,ROW3);
  LCD.printNumF( peak2.read(),2,4*6,ROW4);
  LCD.printNumF( peak3.read(),2,4*6,ROW5);
  */
   LCD.print((char *)"Adc ",0,ROW4);
   LCD.printNumF( peak1.read(),2,4*6,ROW4);
 //  LCD.print((char *)"Sel ",0,ROW5);
 //  LCD.printNumF( peak2.read(),2,4*6,ROW5);
}


void eer_test(){      // !!! debug
static int sec;
static int freq = 4000;
  // Serial.print(sec);   Serial.write(' ');
  // Serial.print(eer_adj); Serial.write(' ');
  // Serial.print(temp_count); Serial.write(' ');
  // Serial.print( eer_time,5 ); Serial.write(' ');
  // Serial.println( overs );
   LCD.printNumF(eer_time,5,0,ROW3);

   eer_adj = 0; overs = 0;
   if( sec == 15 ) sec = 0;
   if( sec == 3 ) tx();
   if( sec == 12 ) rx();
   ++sec;
   SideTone.frequency(freq);
   if ( sec < 12 && sec >= 3 ){
    if( freq <= 1000 ) freq -= 50;
    else freq -= 200;
   }
   if( freq < 300 ) freq = 3600;               // think 4000 was aliasing
}

void button_process( int t ){

    if( transmitting ){
         rx();                   // abort tx on any ? do we want this or just return
         return;
    }
    
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
           step_timer = 1500;                  // 1.5 seconds to dtap up past 1000
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
           step_timer = 1500;
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
   set_af_gain(af_gain);

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

    if( transmitting ) return;  // can't use I2C for other purposes during transmit
    freq = f;
    //    noInterrupts();                            // AM tuned dead on freq is distorted with the square law detector.
    if( mode == AM ) si5351.freq(freq + 1000,0,90);  //  once was freq-9000 to tune one sideband on IF 7k to 11k
    else if(mode == CW){
      si5351.freq(freq + cw_offset, 90, 0);  // RX in LSB
      si5351.freq_calc_fast(-cw_offset); si5351.SendPLLBRegisterBulk(); // TX at freq
    }
    else if(mode == LSB) si5351.freq(freq, 90, 0);  // RX in LSB
    else si5351.freq(freq, 0, 90);  // RX in USB
    //interrupts();
    
    //freq_display();     // need to delay screen update until after menu_cleanup
    
}

void status_display(){
const char modes[] = "CW LSBUSBAM ";
char msg[4];
char msg2[9];
char buf[20];

    if( transmitting ) return;       // avoid OLED writes
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
  //mode = bandstack[band].mode;
  step_ = bandstack[band].stp;
  freq = bandstack[band].freq;
  mode_change(bandstack[band].mode);
//  qsy( bandstack[band].freq );  // done in mode_change
//  status_display();            delay until after screen clear
  
}

void mode_change( int to_mode ){

  if( mode == AM && to_mode != AM) AMoff();
  if( to_mode == AM && mode != AM) AMon();
  mode = to_mode;
  qsy( freq );                                       // to get phasing correct on QSD
  set_af_gain(af_gain);                              // listen to the correct audio path
  set_bandwidth();                                   // bandwidth is mode dependent
 // if( mode == AM ) AMon();
}


// select the audio to listen to: SSB, AM, Sidetone
/************
void audio_select(int m){
const float sel[3][4] = { {0,1.0,-1.0,0}, {1.0,0,0,0}, {0,0,0,0.1} };    // !!! make sidetone volume variable
this is all wrong now !!!
   if( m == LSB || m == USB || m == CW ) m = 0;
   else if( m == AM ) m = 1;
   else m = 2;                    // sidetone
   // else need an off setting ?

   for( int i = 0; i < 4; ++i ) Sub_SSB.gain(i,sel[m][i]);
} *********/

// turn on / off the Fir filters for AM mode
void AMon(){
  AudioNoInterrupts();
  //  Ip90.end();
  //  Qm00.end();
  //  Ip90.begin(AMp90,34);
  //  Qm00.begin(AMm00,34);
  //  AMlow.begin(AMlowpass,30);
  SSB_AM.setmode( 1 ); 
  AudioInterrupts();
}

void AMoff(){
  AudioNoInterrupts();
   // Ip90.end();
   // Qm00.end();
   // AMlow.end();
   // Ip90.begin(SSBp90,64);
   // Qm00.begin(SSBm00,64);
   SSB_AM.setmode( 0 );
  AudioInterrupts();  
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


#define AGC_FLOOR  0.015             // was 0.035, was 0.07
#define AGC_SLOPE  0.9             // was .8
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
      if( encoder_user == FREQ && transmitting == 0 ){
        OLD.printNumF( sig,2,0,ROW6 );            // move these 4 lines to an S meter function
        OLD.printNumI( AudioProcessorUsage(),60,ROW6,3,' ');   // 0 to 100 percent.  More debug
        LCD.print((char *)"Sig ",84-6*6,ROW0);
        LCD.printNumI((int)(sig * 100.0),RIGHT,ROW0,3,' '); 
      }   
       g = 1.0 - sig + AGC_FLOOR;
       set_agc_gain( g * agc_gain );                  // front end gain.  Only manual agc_gain can increase it.
    }                                                 // AGC can only decrease it.
                                                      // may have to adjust FLOOR with manual agc_gain changes.
}                                                     // see how this works first.


int encoder(){         /* read encoder, return 1, 0, or -1 */
static int mod;        /* encoder is divided by 4 because it has detents */
static int dir;        /* need same direction as last time, effective debounce */
static int last;       /* save the previous reading */
int new_;              /* this reading */
int b;

   if( transmitting ) return 0;
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
static int32_t tm;               // catch missing ticks if any
int sw;
int t;

      if( fini ){                // switch state latched until processed and we say done
          st = DONE;
          return st;
      }
      t = (int)(millis() - tm);
      tm = millis();
      sw = digitalReadFast(EN_SW) ^ 1;   
      if( sw ) press_ += t, nopress= 0;
      else nopress += t, press_= 0;
      
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
   5,
   "Select Mode",
   { "CW", "LSB", "USB", "AM", "Mem Tune" }          
};


struct MENU band_menu = {
   6,
   "Amateur Band",
   {"80", "60", "40", "30", "20", "17" }
};

struct MENU bandwidth_menu = {
    6,
    "Band Width",
    {"Narrow", "CW-Wide", "2700", "3000", "3300", "3600" }
};

struct MENU main_menu = {
  3,
  "Top Menu",
  { "Band", "Mode", "Filter" }
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
              case 2: active_menu = &bandwidth_menu; def_val = filter; state = 3; break;
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
         case 3:
            filter = def_val;
            set_bandwidth();
            ret_val = state = 0;
         break;
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


#ifdef NOWAY
/***********************   saving some old code   */

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
  Wire.begin(I2C_OP_MODE_DMA);   // use mode DMA or ISR ?, dma takes a long time to reset the OLED. Both seem to work.
  Wire.setClock(600000);     // I2C0_F  40 = 100k ,  25 = 400k.  800000 seems to work, returns 818, 600k returns 600
                             // clock stretching may produce reduced speed if clock is way to fast for the devices.
                             // may need a speed test or scope the scl sda to see what is really going on.
  //I2C0_F = 15;             // override standard speeds?  Now using i2c_t3 library, it supports non-standard speeds 
}

void i2start( unsigned char adr ){

  while( Wire.done() == 0 );      // still busy with last.  Need to block while still busy???.  Did we gain anything from std Wire?
  Wire.beginTransmission( adr );

  // could double buffer and use the callback ( Wire.onTransmitDone(function); ) to send the next set of buffered data up to 
  // the next stop.  But it seems that could be tricky when buffers empty and callback has nothing to do, or when the buffer fills.

unsigned int dat;
  // shift the address over and add the start flag
  dat = ( adr << 1 ) | ISTART;
  i2send( dat ); */
 }

void i2send( unsigned int data ){   // just save stuff in the buffer
int next;

  Wire.write( data );

  next = (i2in + 1) & (I2BUFSIZE - 1);
  while( i2out == next ) i2poll();
  i2buf[i2in++] = data;
  i2in &= (I2BUFSIZE - 1);
  //i2poll();   // !!! did this cause an error?
  
}


void i2stop( ){
  Wire.sendTransmission();     // non-blocking
 // Wire.endTransmission();      // blocking
  // i2send( ISTOP );   // que a stop condition
}


void i2flush(){  //  call poll to empty out the buffer.  This one does block.

  while( i2poll() ); 
}

// save this, might be only copy of these direct to hardware Teensy I2C routines.
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
/*
  * peak magnitude estimation ( derek rowell )
  *   0.960433870103 * maxIQ + 0.3978247347593 * minIQ
  *   Testing
  *   sqrt ( 12*12 + 7*7 ) is 13.892
  *   0.9604 * 12 + 0.3978 * 7 is 14.3
  *   Q15 version: 31470 * 12  +  13035 * 7   =  468,885 then shift by 15 = 14 ( 1110 )
  *   
  *   QCX-SSB method:  maxIQ + 1/4 minIQ
  *   12 + 7/4 = 13
  *   bigger numbers
  *      314  223    sqrt --> 385
  *      314 + 223/4 = 369
  *  Q15             = 390
  *      314 + 223/4 + 223/8 = 397
  *      314 - 314/32 + 223/4 + 223/8 = 387
  *      314 * 31 / 32 + 223 * 3 / 8  using shifts for dividing may be promising, 2 mults, 2 shifts, 1 add.
  *      (31 * 314) >> 5 + (3 * 223) >> 3    try this one.  
  *      (abs(I) * 31) >> 5 + (abs(Q) * 3 ) >> 3 where I > Q
  */


#endif
 
