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
 *    
 */

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
  *      (31 * 314) >> 5 + (3 * 223) >> 3
  *      ? would this remove AM carrier.  ie satisfy sin()^2 + cos()^2 = 1
  *      ? need to highpass the result to remove DC generated from carrier.
  */

#define VERSION 1.37

/*
 * The encoder switch works like this:
 * A tap changes the tuning step with 1k, 100, 10 in rotation.
 *   To change to a larger tuning step, first tap then double tap within 1.5 seconds and select 5k, 50k, 500k steps with double taps.
 * Normally a double tap brings up the volume control.  When done adjusting, double tap back to normal turning.
 * Long Press brings up the menu.  You can make selections with a tap.  Double tap exits.  And see sticky menus option below for 
 *   different behavior. 
 */

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

// #include "filters.h"

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
#include "AM_decoder.h"

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
#define AM2 4 
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
int filter = 4;
int bfo;

int step_timer;                // allows double tap to backup the freq step to 500k , command times out
                               // and returns to the normal double tap command ( volume )
float af_gain = 0.3;
float agc_gain = 1.0;           // above 1 perhaps not a good idea.
float sig_usb;

/******************************** Teensy Audio Library **********************************/ 

#include <Audio.h>
//#include <Wire.h>
//#include <SPI.h>
//#include <SD.h>
//#include <SerialFlash.h>

// Weaver with bi-quads.  Tx sources, USB added.  Separate AGC amp removed.  AM decode library object.


// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=98.5714340209961,246.4285707473755
//AudioInputUSB            usb2;           //xy=102.5714340209961,309.4285707473755
AudioAnalyzePeak         peak1;          //xy=268.5714340209961,91.42857074737549
AudioMixer4              RxTx1;          //xy=288.5714340209961,169.4285707473755
AudioMixer4              RxTx2;          //xy=291.5714340209961,340.4285707473755
AudioFilterBiquad        I_filter;       //xy=424.5714340209961,190.4285707473755
AudioAnalyzePeak         peak2;          //xy=427.5714340209961,94.42857074737549
AudioFilterBiquad        Q_filter;       //xy=428.5714340209961,324.4285707473755
AudioSynthWaveformSine   cosBFO;         //xy=527.5714340209961,239.4285707473755
AudioSynthWaveformSine   sinBFO;         //xy=531.5714340209961,286.4285707473755
AudioEffectMultiply      I_mixer;        //xy=630.5714340209961,195.4285707473755
AudioEffectMultiply      Q_mixer;        //xy=634.5714340209961,331.4285707473755
AudioAMdecode2           AMdecode2;      //xy=642.5714340209961,410.4285707473755
AudioMixer4              Sub_SSB;        //xy=769.5714340209961,251.4285707473755
AudioFilterBiquad        AMFilter;        //xy=834.2857246398926,410.0000047683716
AudioAnalyzeRMS          rms1;           //xy=889.5714340209961,161.4285707473755
AudioMixer4              Volume;         //xy=927.1428571428571,262.85714285714283
//AudioOutputUSB           usb1;           //xy=1055.571434020996,301.4285707473755
AudioOutputAnalog        dac1;           //xy=1060.571434020996,251.4285707473755
AudioConnection          patchCord1(adcs1, 0, RxTx1, 0);
AudioConnection          patchCord2(adcs1, 0, RxTx1, 1);
AudioConnection          patchCord3(adcs1, 0, RxTx2, 1);
AudioConnection          patchCord4(adcs1, 0, peak1, 0);
AudioConnection          patchCord5(adcs1, 1, RxTx2, 0);
//AudioConnection          patchCord6(usb2, 0, RxTx1, 2);
//AudioConnection          patchCord7(usb2, 0, RxTx2, 2);
AudioConnection          patchCord8(RxTx1, I_filter);
AudioConnection          patchCord9(RxTx1, peak2);
AudioConnection          patchCord10(RxTx2, Q_filter);
AudioConnection          patchCord11(I_filter, 0, I_mixer, 0);
AudioConnection          patchCord12(I_filter, 0, AMdecode2, 0);
AudioConnection          patchCord13(Q_filter, 0, Q_mixer, 0);
AudioConnection          patchCord14(Q_filter, 0, AMdecode2, 1);
AudioConnection          patchCord15(cosBFO, 0, I_mixer, 1);
AudioConnection          patchCord16(sinBFO, 0, Q_mixer, 1);
AudioConnection          patchCord17(I_mixer, 0, Sub_SSB, 1);
AudioConnection          patchCord18(Q_mixer, 0, Sub_SSB, 2);
AudioConnection          patchCord19(AMdecode2, AMFilter);
AudioConnection          patchCord20(Sub_SSB, rms1);
AudioConnection          patchCord21(Sub_SSB, 0, Volume, 0);
AudioConnection          patchCord22(AMFilter, 0, Volume, 1);
AudioConnection          patchCord23(Volume, dac1);
//AudioConnection          patchCord24(Volume, 0, usb1, 0);
//AudioConnection          patchCord25(Volume, 0, usb1, 1);
// GUItool: end automatically generated code


/*
// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=74.28574752807617,447.1429252624512
AudioInputUSB            usb2;           //xy=78.57145690917969,510.0000228881836
AudioAnalyzePeak         peak1;          //xy=244.4285888671875,292.0000305175781
AudioMixer4              RxTx1;         //xy=264.2857246398926,370.00003719329834
AudioMixer4              RxTx2;         //xy=267.14288330078125,541.428599357605
AudioFilterBiquad        I_filter;        //xy=400.8928756713867,391.2500104904175
AudioAnalyzePeak         peak2;          //xy=403,295
AudioFilterBiquad        Q_filter;        //xy=404.6428756713867,525.0000104904175
AudioSynthWaveformSine   cosBFO;         //xy=503.00000381469727,440.00002098083496
AudioSynthWaveformSine   sinBFO;         //xy=507.28575897216797,487.5713748931885
AudioEffectMultiply      I_mixer;        //xy=606.7143173217773,396.14284896850586
AudioEffectMultiply      Q_mixer;        //xy=610.7143096923828,532.7142696380615
AudioMixer4              MysteryObject;         //xy=618.5715065002441,611.4285755157471
AudioMixer4              Sub_SSB;        //xy=745.2857513427734,452.00001430511475
AudioAnalyzeRMS          rms1;           //xy=865.7143783569336,362.28568840026855
AudioAmplifier           Volume;           //xy=890.0000457763672,452.8572006225586
AudioOutputUSB           usb1;           //xy=1031.4286308288574,502.8572196960449
AudioOutputAnalog        dac1;           //xy=1036.1427917480469,452.00001335144043
AudioConnection          patchCord1(adcs1, 0, RxTx1, 0);
AudioConnection          patchCord2(adcs1, 0, RxTx1, 1);
AudioConnection          patchCord3(adcs1, 0, RxTx2, 1);
AudioConnection          patchCord4(adcs1, 0, peak1, 0);
AudioConnection          patchCord5(adcs1, 1, RxTx2, 0);
AudioConnection          patchCord6(usb2, 0, RxTx1, 2);
AudioConnection          patchCord7(usb2, 0, RxTx2, 2);
AudioConnection          patchCord8(RxTx1, I_filter);
AudioConnection          patchCord9(RxTx1, peak2);
AudioConnection          patchCord10(RxTx2, Q_filter);
AudioConnection          patchCord11(I_filter, 0, I_mixer, 0);
AudioConnection          patchCord12(I_filter, 0, MysteryObject, 0);
AudioConnection          patchCord13(Q_filter, 0, Q_mixer, 0);
AudioConnection          patchCord14(Q_filter, 0, MysteryObject, 1);
AudioConnection          patchCord15(cosBFO, 0, I_mixer, 1);
AudioConnection          patchCord16(sinBFO, 0, Q_mixer, 1);
AudioConnection          patchCord17(I_mixer, 0, Sub_SSB, 1);
AudioConnection          patchCord18(Q_mixer, 0, Sub_SSB, 2);
AudioConnection          patchCord19(Sub_SSB, Volume);
AudioConnection          patchCord20(Sub_SSB, rms1);
AudioConnection          patchCord21(Volume, dac1);
AudioConnection          patchCord22(Volume, 0, usb1, 0);
AudioConnection          patchCord23(Volume, 0, usb1, 1);
// GUItool: end automatically generated code
*/
/*
// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=74.28574752807617,447.1429252624512
//AudioInputUSB            usb2;           //xy=78.57145690917969,510.0000228881836
AudioAnalyzePeak         peak1;          //xy=244.4285888671875,292.0000305175781
AudioMixer4              RxTx1;         //xy=264.2857246398926,370.00003719329834
AudioMixer4              RxTx2;         //xy=267.14288330078125,541.428599357605
AudioFilterBiquad        I_filter;        //xy=400.8928756713867,391.2500104904175
AudioAnalyzePeak         peak2;          //xy=403,295
AudioFilterBiquad        Q_filter;        //xy=404.6428756713867,525.0000104904175
AudioSynthWaveformSine   cosBFO;         //xy=513,437.1428737640381
AudioSynthWaveformSine   sinBFO;         //xy=515.8572044372559,480.42853832244873
AudioEffectMultiply      I_mixer;        //xy=606.7143096923828,389.0000104904175
AudioEffectMultiply      Q_mixer;        //xy=610.7143096923828,527.0000104904175
AudioMixer4              MysteryObject;         //xy=618.5715065002441,611.4285755157471
AudioMixer4              Sub_SSB;        //xy=745.2857513427734,452.00001430511475
AudioAmplifier           Volume;           //xy=838.5714836120605,562.8571844100952
AudioAmplifier           agc_amp;           //xy=918.571418762207,452.8571481704712
AudioAnalyzeRMS          rms1;           //xy=965.7143096923828,368.0000104904175
AudioOutputAnalog        dac1;           //xy=984.7143096923828,562.0000104904175
//AudioOutputUSB           usb1;           //xy=990.0000228881836,604.2857370376587
AudioConnection          patchCord1(adcs1, 0, RxTx1, 0);
AudioConnection          patchCord2(adcs1, 0, RxTx1, 1);
AudioConnection          patchCord3(adcs1, 0, RxTx2, 1);
AudioConnection          patchCord4(adcs1, 0, peak1, 0);
AudioConnection          patchCord5(adcs1, 1, RxTx2, 0);
//AudioConnection          patchCord6(usb2, 0, RxTx1, 2);
//AudioConnection          patchCord7(usb2, 0, RxTx2, 2);
AudioConnection          patchCord8(RxTx1, I_filter);
AudioConnection          patchCord9(RxTx1, peak2);
AudioConnection          patchCord10(RxTx2, Q_filter);
AudioConnection          patchCord11(I_filter, 0, I_mixer, 0);
AudioConnection          patchCord12(I_filter, 0, MysteryObject, 0);
AudioConnection          patchCord13(Q_filter, 0, Q_mixer, 0);
AudioConnection          patchCord14(Q_filter, 0, MysteryObject, 1);
AudioConnection          patchCord15(cosBFO, 0, I_mixer, 1);
AudioConnection          patchCord16(sinBFO, 0, Q_mixer, 1);
AudioConnection          patchCord17(I_mixer, 0, Sub_SSB, 1);
AudioConnection          patchCord18(Q_mixer, 0, Sub_SSB, 2);
AudioConnection          patchCord19(Sub_SSB, agc_amp);
AudioConnection          patchCord20(Volume, dac1);
//AudioConnection          patchCord21(Volume, 0, usb1, 0);
//AudioConnection          patchCord22(Volume, 0, usb1, 1);
AudioConnection          patchCord23(agc_amp, Volume);
AudioConnection          patchCord24(agc_amp, rms1);
// GUItool: end automatically generated code
*/
/*
// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=74.28574752807617,447.1429252624512
//AudioInputUSB            usb2;           //xy=78.57145690917969,510.0000228881836
AudioMixer4              RxTx1;         //xy=264.2857246398926,370.00003719329834
AudioMixer4              RxTx2;         //xy=267.14288330078125,541.428599357605
AudioAnalyzePeak         peak1;          //xy=391.42860412597656,300.00003147125244
AudioFilterBiquad        I_filter;        //xy=400.8928756713867,391.2500104904175
AudioFilterBiquad        Q_filter;        //xy=404.6428756713867,525.0000104904175
AudioSynthWaveformSine   cosBFO;         //xy=513,437.1428737640381
AudioSynthWaveformSine   sinBFO;         //xy=515.8572044372559,480.42853832244873
AudioEffectMultiply      I_mixer;        //xy=606.7143096923828,389.0000104904175
AudioEffectMultiply      Q_mixer;        //xy=610.7143096923828,527.0000104904175
AudioMixer4              MysteryObject;         //xy=618.5715065002441,611.4285755157471
AudioMixer4              Sub_SSB;        //xy=745.2857513427734,452.00001430511475
AudioAmplifier           Volume;           //xy=838.5714836120605,562.8571844100952
AudioAmplifier           agc_amp;           //xy=918.571418762207,452.8571481704712
AudioAnalyzeRMS          rms1;           //xy=965.7143096923828,368.0000104904175
AudioOutputAnalog        dac1;           //xy=984.7143096923828,562.0000104904175
//AudioOutputUSB           usb1;           //xy=990.0000228881836,604.2857370376587
AudioConnection          patchCord1(adcs1, 0, RxTx1, 0);
AudioConnection          patchCord2(adcs1, 0, RxTx1, 1);
AudioConnection          patchCord3(adcs1, 0, RxTx2, 1);
AudioConnection          patchCord4(adcs1, 1, RxTx2, 0);
//AudioConnection          patchCord5(usb2, 0, RxTx1, 2);
//AudioConnection          patchCord6(usb2, 0, RxTx2, 2);
AudioConnection          patchCord7(RxTx1, I_filter);
AudioConnection          patchCord8(RxTx1, peak1);
AudioConnection          patchCord9(RxTx2, Q_filter);
AudioConnection          patchCord10(I_filter, 0, I_mixer, 0);
AudioConnection          patchCord11(I_filter, 0, MysteryObject, 0);
AudioConnection          patchCord12(Q_filter, 0, Q_mixer, 0);
AudioConnection          patchCord13(Q_filter, 0, MysteryObject, 1);
AudioConnection          patchCord14(cosBFO, 0, I_mixer, 1);
AudioConnection          patchCord15(sinBFO, 0, Q_mixer, 1);
AudioConnection          patchCord16(I_mixer, 0, Sub_SSB, 1);
AudioConnection          patchCord17(Q_mixer, 0, Sub_SSB, 2);
AudioConnection          patchCord18(Sub_SSB, agc_amp);
AudioConnection          patchCord19(Volume, dac1);
// AudioConnection          patchCord20(Volume, 0, usb1, 0);
//AudioConnection          patchCord21(Volume, 0, usb1, 1);
AudioConnection          patchCord22(agc_amp, Volume);
AudioConnection          patchCord23(agc_amp, rms1);
// GUItool: end automatically generated code
*/
/*   weaver with biquads
// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=118.5714340209961,450
AudioAnalyzePeak         peak1;          //xy=354.28572845458984,300.00002098083496
AudioFilterBiquad        I_filter;        //xy=363.75,391.25
AudioFilterBiquad        Q_filter;        //xy=367.5,525
AudioSynthWaveformSine   cosBFO;         //xy=481.5714340209961,460
AudioSynthWaveformSine   sinBFO;         //xy=481.5714340209961,609
AudioEffectMultiply      I_mixer;        //xy=569.5714340209961,389
AudioEffectMultiply      Q_mixer;        //xy=573.5714340209961,527
AudioMixer4              Sub_SSB;        //xy=708.1428756713867,452.00000381469727
AudioAmplifier           Volume;           //xy=801.4286079406738,562.8571739196777
AudioFilterBiquad        BandWidth;        //xy=861.428581237793,452.857102394104
AudioAnalyzeRMS          rms1;           //xy=928.5714340209961,368
AudioOutputAnalog        dac1;           //xy=947.5714340209961,562
AudioAmplifier           agc_amp;           //xy=1017.1428642272949,452.857141494751
AudioConnection          patchCord1(adcs1, 0, I_filter, 0);
AudioConnection          patchCord2(adcs1, 1, peak1, 0);
AudioConnection          patchCord3(adcs1, 1, Q_filter, 0);
AudioConnection          patchCord4(I_filter, 0, I_mixer, 0);
AudioConnection          patchCord5(Q_filter, 0, Q_mixer, 0);
AudioConnection          patchCord6(cosBFO, 0, I_mixer, 1);
AudioConnection          patchCord7(sinBFO, 0, Q_mixer, 1);
AudioConnection          patchCord8(I_mixer, 0, Sub_SSB, 1);
AudioConnection          patchCord9(Q_mixer, 0, Sub_SSB, 2);
AudioConnection          patchCord10(Sub_SSB, BandWidth);
AudioConnection          patchCord11(Volume, dac1);
AudioConnection          patchCord12(BandWidth, agc_amp);
AudioConnection          patchCord13(agc_amp, Volume);
AudioConnection          patchCord14(agc_amp, rms1);
// GUItool: end automatically generated code
*/
/*
// Weaver with FIR filters.  Didn't work well at all.

// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=118.5714340209961,450
AudioFilterFIR           Q_filter;           //xy=348.5714225769043,521.4285297393799
AudioFilterFIR           I_filter;           //xy=350.0000114440918,382.85715675354004
AudioSynthWaveformSine   cosBFO;         //xy=481.5714340209961,460
AudioSynthWaveformSine   sinBFO;         //xy=481.5714340209961,609
AudioEffectMultiply      I_mixer;        //xy=569.5714340209961,389
AudioEffectMultiply      Q_mixer;        //xy=573.5714340209961,527
AudioMixer4              Sub_SSB;        //xy=708.1428756713867,452.00000381469727
AudioAmplifier           Volume;           //xy=801.4286079406738,562.8571739196777
AudioFilterBiquad        BandWidth;        //xy=861.428581237793,452.857102394104
AudioAnalyzeRMS          rms1;           //xy=928.5714340209961,368
AudioOutputAnalog        dac1;           //xy=947.5714340209961,562
AudioAmplifier           agc_amp;           //xy=1017.1428642272949,452.857141494751
 AudioAnalyzePeak         peak1;          //xy=322.71430587768555,410.42852210998535
 AudioConnection          patchCord14(adcs1, 1, peak1, 0);
AudioConnection          patchCord1(adcs1, 0, I_filter, 0);
AudioConnection          patchCord2(adcs1, 1, Q_filter, 0);
AudioConnection          patchCord3(Q_filter, 0, Q_mixer, 0);
AudioConnection          patchCord4(I_filter, 0, I_mixer, 0);
AudioConnection          patchCord5(cosBFO, 0, I_mixer, 1);
AudioConnection          patchCord6(sinBFO, 0, Q_mixer, 1);
AudioConnection          patchCord7(I_mixer, 0, Sub_SSB, 1);
AudioConnection          patchCord8(Q_mixer, 0, Sub_SSB, 2);
AudioConnection          patchCord9(Sub_SSB, BandWidth);
AudioConnection          patchCord10(Volume, dac1);
AudioConnection          patchCord11(BandWidth, agc_amp);
AudioConnection          patchCord12(agc_amp, Volume);
AudioConnection          patchCord13(agc_amp, rms1);
// GUItool: end automatically generated code
*/
/*
     Hilbert design.
// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=118.57144165039062,535.7142238616943
// AudioInputUSB            usb1;           //xy=118.57145690917969,615.7143611907959
AudioMixer4              RxTx2;         //xy=318.5714302062988,595.7142715454102
AudioMixer4              RxTx1;         //xy=320.00000381469727,481.42858505249023
AudioAnalyzePeak         peak1;          //xy=322.71430587768555,410.42852210998535
AudioFilterFIR           h45m;           //xy=455.7143096923828,596.714241027832
AudioFilterFIR           h45p;           //xy=457.7143096923828,480.71424102783203
AudioMixer4              Add_SSB;        //xy=636.7143096923828,535.714241027832
AudioMixer4              Mystery_Object;         //xy=664.285774230957,658.571475982666
AudioAmplifier           Volume;           //xy=800.0001373291016,605.7143096923828
AudioAnalyzeRMS          rms1;           //xy=809.9999923706055,446.57135581970215
AudioAmplifier           agc_amp;           //xy=809.7143096923828,535.714241027832
AudioFilterBiquad        BandWidth;        //xy=941.4284896850586,605.7143707275391
AudioOutputAnalog        dac1;           //xy=1012.0001373291016,466.28564262390137
//AudioOutputUSB           usb2;           //xy=1041.4284896850586,512.8570823669434
AudioConnection          patchCord1(adcs1, 0, RxTx1, 0);
AudioConnection          patchCord2(adcs1, 0, RxTx1, 1);
AudioConnection          patchCord3(adcs1, 0, RxTx2, 1);
AudioConnection          patchCord4(adcs1, 1, RxTx2, 0);
AudioConnection          patchCord5(adcs1, 1, peak1, 0);
//AudioConnection          patchCord6(usb1, 0, RxTx1, 2);
//AudioConnection          patchCord7(usb1, 0, RxTx2, 2);
AudioConnection          patchCord8(RxTx2, h45m);
AudioConnection          patchCord9(RxTx1, h45p);
AudioConnection          patchCord10(h45m, 0, Add_SSB, 3);
AudioConnection          patchCord11(h45m, 0, Mystery_Object, 1);
AudioConnection          patchCord12(h45p, 0, Add_SSB, 0);
AudioConnection          patchCord13(h45p, 0, Mystery_Object, 0);
AudioConnection          patchCord14(Add_SSB, agc_amp);
AudioConnection          patchCord15(Volume, BandWidth);
AudioConnection          patchCord16(agc_amp, rms1);
AudioConnection          patchCord17(agc_amp, Volume);
AudioConnection          patchCord18(BandWidth, dac1);
//AudioConnection          patchCord19(BandWidth, 0, usb2, 0);
//AudioConnection          patchCord20(BandWidth, 0, usb2, 1);
// GUItool: end automatically generated code

*/

// I2C functions that the OLED library expects to use.
void i2init(){

  Wire.begin(I2C_OP_MODE_DMA);   // use mode DMA or ISR ?, dma takes a long time to reset the OLED. Both seem to work.
  Wire.setClock(700000);     // I2C0_F  40 = 100k ,  25 = 400k.  800000 seems to work, returns 818, 600k returns 600
                             // clock stretching may produce reduced speed if clock is way to fast for the devices.
                             // may need a speed test or scope the scl sda to see what is really going on.
                             // Calc that 700k speed could maybe support a tx bandwidth of 5.5k at 11029 sample rate.
}

void i2start( unsigned char adr ){

  while( Wire.done() == 0 );      // still busy with last.  Need to block while still busy???.  Did we gain anything from std Wire?
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

  //Serial.print( fout ); Serial.write(' ');
  //Serial.print( fvcoa ); Serial.write(' ');
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
  void powerDown(){
    for(int addr = 16; addr != 24; addr++) SendRegister(addr, 0b10000000);  // Conserve power when output is disabled
    SendRegister(3, 0b11111111); // Disable all CLK outputs    
  }
  #define SI_CLK_OE 3 

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
     OLD.printNumI( Wire.getClock()/1000,0,ROW4);
   #endif

   freq_display();
   status_display();
   encoder();             // pick up current position
   
  fake_s();

// Audio Library setup
  AudioNoInterrupts();
  AudioMemory(40);
  
  Sub_SSB.gain(1,1.0);   // correct sideband
  Sub_SSB.gain(2,-1.0);
  Sub_SSB.gain(0,0.0);
  Sub_SSB.gain(3,0.0);

  RxTx1.gain(0,1.0);     // audio mux's
  RxTx2.gain(0,1.0);     // Rx = 0, Tx mic = 1, Tx usb = 2
  RxTx1.gain(1,0.0);
  RxTx1.gain(2,0.0);
  RxTx1.gain(3,0.0);
  RxTx2.gain(1,0.0);
  RxTx2.gain(2,0.0);
  RxTx2.gain(3,0.0);

  // AM post multirate filter
  // 0.54119610   butterworth Q's two cascade
  // 1.3065630
  AMFilter.setHighpass(0,300,0.54119610);    // hipass to remove DC level
  AMFilter.setHighpass(1,300,1.3065630);
  AMFilter.setLowpass(2,4000,0.54119610);    // lowpass for multirate upsample 11k to 44k
  AMFilter.setLowpass(3,4000,1.3065630);
 

  set_af_gain(af_gain);
  set_agc_gain(agc_gain);
  
//  set_bandwidth( 5000 );      // removed object
//  I_filter.begin(fir3k,30);    // min phase kaiser 3k
//  Q_filter.begin(fir3k,30);

  // set_Weaver_bandwidth(3300);
  filter = 4;
  set_rx_bandwidth();

  AudioInterrupts();

}

/*  ******************
void set_bandwidth( int bw ){

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
     BandWidth.setLowpass(0,750+bw/2,0.54119610);
     BandWidth.setLowpass(1,750+bw/2,1.3065630);       // Butterworth Q's for 2 cascade
     BandWidth.setHighpass(2,750-bw/2,0.54119610);
     BandWidth.setHighpass(3,750-bw/2,1.3065630);    
  }
  
} *********************  */

void set_af_gain(float g){

  if( mode >= AM ){
     Volume.gain(0,0.0);
     Volume.gain(1,2*g);
  }
  else{
     Volume.gain(1,0.0);
     Volume.gain(0,g);    
  }

  
}

void set_agc_gain(float g ){
   // try front end gain agc again
   // agc_amp.gain(g);
   //agc_amp.gain(1.0);
  AudioNoInterrupts();
    RxTx1.gain(0,g);     // audio mux's
    RxTx2.gain(0,g);     // Rx = 0, Tx mic = 1, Tx usb = 2
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
       case 0: bw = 2500; break;     // data duplicated from menu strings.  Could be improved so that
       case 1: bw = 2500; break;     // one doesn't need to maintain data in two places if changes are made.
       case 2: bw = 2700; break;
       case 3: bw = 3000; break;
       case 4: bw = 3300; break;
       case 5: bw = 3600; break;
   }

  if( mode == AM || mode == AM2 ) bw = 7000;   // full signal to AM decoder as Weaver not used
  bfo = bw/2;                           // weaver audio folding at 1/2 bandwidth

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


void tx(){
  // what needs to change to enter tx mode
  // audio mux switching, tx bandwidth, pwm, SI5351 clocks on/off, I2C speed change, ...
  
}

void rx(){
  // what needs to change to return to rx mode.
  
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

   // i2poll();

   //  1 ms routines
   if( tm != millis()){ 
 //t = (int) (millis() - tm);
 // OLD.printNumI(t,RIGHT,ROW6,3,' ');
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
      // report_peaks();
   }

   if( t2 ){
     --t2;
     if( t2 == 1 ){
        sig_usb = rms1.read();
        agc_process( sig_usb);
        report_peaks();
     }
    // if( t2 == 0 && mode == AM ) aft_process();
   }
   
}

void report_peaks(){
static int count;

  if( ++count < 1000 ) return;            // once a second
  if( encoder_user != FREQ ) return;
  count = 0;

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
   LCD.print((char *)"Sel ",0,ROW5);
   LCD.printNumF( peak2.read(),2,4*6,ROW5);
}


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

    // with weaver rx, freq is the display frequency.  vfo and bfo move about with bandwidth changes.
    freq = f;
    //    noInterrupts();
    if( mode >= AM ) si5351.freq(freq+1500,0,90);       // tune one sideband
    else if(mode == CW){
      si5351.freq(freq + cw_offset - bfo, 90, 0);  // RX in LSB
      si5351.freq_calc_fast(-cw_offset + bfo); si5351.SendPLLBRegisterBulk(); // TX at freq
    }
    else if(mode == LSB) si5351.freq(freq - bfo, 90, 0);  // RX in LSB
    else si5351.freq(freq + bfo, 0, 90);  // RX in USB, AM
    //interrupts();
    
    //freq_display();     // need to delay screen update until after menu_cleanup
    
}

void status_display(){
const char modes[] = "CW LSBUSBAM AM2";
char msg[4];
char msg2[9];
char buf[20];
    
    if( mode > 4 ) return;           //!!! how to handle memory tuning
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
  qsy( freq );                                       // to get phasing correct
  if( mode == AM ) AMdecode2.setmode(1);           //  1 = fast code, 2 = more exact slower code
  else if( mode == AM2 ) AMdecode2.setmode(2);
  else AMdecode2.setmode(0);
  set_af_gain(af_gain);
  set_rx_bandwidth();
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
      if( encoder_user == FREQ ){
        OLD.printNumF( sig,2,0,ROW6 );            // move these 4 lines to an S meter function
        OLD.printNumI( AudioProcessorUsage(),60,ROW6,3,' ');   // 0 to 100 percent.  More debug
        LCD.print((char *)"Sig ",84-6*6,ROW0);
        LCD.printNumI((int)(sig * 100.0),RIGHT,ROW0,3,' '); 
      }   
       g = 1.0 - sig + AGC_FLOOR;
       set_agc_gain( g * agc_gain );                    // back end gain.  Only manual agc_gain can increase it.
    }                                                 // AGC can only decrease it.
                                                      // may have to adjust FLOOR with manual agc_gain changes.
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
   7,
   "Select Mode",
   { "CW", "LSB", "USB", "AM", "AM2", "Mem Tune", "WSPR" }          
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
            set_rx_bandwidth();
           // set_Weaver_bandwidth( bw );
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


#endif
 
