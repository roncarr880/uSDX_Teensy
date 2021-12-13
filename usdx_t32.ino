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
 *                  Teensyduino install to fix.
 *    Version 1.40  Wrote MagPhase object to extract data from the Audio library stream to use for transmitting.  Set _UA at             
 *                  44117/8 for initial testing.
 *    Version 1.41  I2C did not support sample rate 1/4 of 44k ( 11k ) for TX.  Changed MagPhase to use 1/6 rate or  7352. It does           
 *                  not divide evenly into 128 size buffer, but seems to be working ok.
 *    Version 1.42  Added another custom audio block ( SSB_AM ) to process receive signals at 1/4 rate with classic Hilbert. 
 *                  The upsample/filter seems to lose quite a bit of audio power.  Adjusted agc, and more volume is required.   
 *    Version 1.43  Replaced the FIR filters with Bi-quad and changed SSB_AM to run at 1/3 rate ( 1/3 of 44117 ).  Forming the           
 *                  opinion that Hilberts do not work well at baseband with a high sample rate.
 *    Version 1.44  Since the RX hilberts could not be used for transmitting, returning to the Weaver receive method. The custom
 *                  audio object SSB_AM is no longer needed.  No AM mode for now.
 *    Version 1.45  Added an AM detector.  Double reset the Si5351 on band/mode changes as sometimes it seems out of sync.  Think              
 *                  this issue started when I added the dividers to the bandstack ( to run the Si5351 in spec for bands other than 80). 
 *    Version 1.46  CAT control using Argonaut V emulation.  Added USA band limits indication.  Made some changes to MagPhase, phase
 *                  may or may not be better, magnitude is better. Added CW decoder. Added Digi mode that filters magnitude and phase
 *                  on TX.  Changed _UA to be the TX sample rate as that seems simplest overall.  Added S meter. CW decoder needs help.
 *    Version 1.47  RIT added that works as described below in encoder switch usage. Changed swapping sidebands to the SSB adder
 *                  rather than in the Si5351. 
 *    Version 1.48  Wired DIT, DAH, RX, KEYOUT signals to the Teensy 3.2.  First transmit test sucessful.  Added CW keyer code.  Added 
 *                  attenuator ( tx/rx FET ).  
 *    Version 1.49  Many little changes with menu and cw keyer, speed, practice mode, sidetone, etc. 
 *    Version 1.50  CW keying had pops on one side of the paddle only, probably due to the 1uf I put in place of the 220nf.  Added a
 *                  manual jumper to isolate the nets when in CW mode and connect them in SSB mode. Moved the Si5351 code into
 *                  a separate file. Added a BandWidth object for CW but signals for all modes pass through it, so the Weaver lowpass
 *                  filters are set as a roofing type filter and the new Bandwidth object is used for the final bandwidth control.
 *                  Added a tone control that changes the Q of the Bandwidth object to vary the tone.
 *    Version 1.51  Adding a gain of 10 in front of the CW tone detect object greatly improved the CW decoder.  Added a test to just            
 *                  load 4 registers in SendPLLBRegisterBulk when register 3 was the same as last time. 
 *    Version 1.52  Using a Goertzel filter as a simple FFT scope.  Seems to work ok although update rate is slow.  Cleaned up some of       
 *                  the old code that was commented out as we seem to have settled upon a design.  For DIGI mode, increased Weaver
 *                  bandwidth to put the hole in the audio at 3000 hz. Cut in some low pass filters for other bands in series with the
 *                  17 meter lowpass.  Noticed I now get some interference lines on the WSJT waterfall.  Removed the alternate low pass
 *                  filters provision as transmit current was excessive on some bands.  So back to 17 meters tx/rx and just rx on the 
 *                  lower bands. Using a total of 4 Goertzel filters in the bandscope which makes it run 4 times faster than before. 
 *    Version 1.53  Found a dynamic cassette style microphone in my junk collection.  It has a mono plug so am adding an option to detect
 *                  if DIT is shorted on power up and disable the hardware keyer and PTT.  The idea is to use capacitive touch as the
 *                  keyer/PTT when this condition is detected.  Building a microphone preamp circuit in the footprint of IC10 and 
 *                  abandon the existing DVM circuit for the Teensy version. ( the DVM curcuit remains on the QCX board if I ever wish to 
 *                  use the ATMega328 again ). Preamp design from http://www.zen22142.zen.co.uk/Circuits/Audio/lf071_mic.htm with a few of
 *                  the 10uf caps changed to 1uf.  Decided to move the microphone to the CAT jack, and wired the Teensy version separate 
 *                  from the original uSDX design with the microphone and paddle using the same jack.  Worked on the magnitude and phase
 *                  some more attempting to suppress spikes in phase when amplitude is low.  Mic Tx audio is terrible at this point.
 *    Version 1.54  Reducing USB audio gain below 1.0 as a potential fix for strange program hangup. ( voice audio on USB with max PC                
 *                  microphone volume.  Doesn't hang when PC microphone volume is less than 100 )  Reducing agc1 volume below 1.0 to make 
 *                  sure the following IIR filters do not get overdriven ( voice mode using microphone ).  Trying 0.98 for a value.
 *                  Add a feature to allow delaying the phase changes with respect to the amplitude changes. uSDX has a delay of 1 sample,
 *                  allow a choice of delay of up to 7 samples. Implemented tx_drive after the MagPhase calculation rather than before. 
 *                  Note: uSDX also adds a dc offset of 32 to microphone adc values and that is something we can try.
 *                  
 *                 
 *                  
 *                  
 *                  
 *             
 */

#define VERSION 1.54

// Paddle jack has Dah on the Tip and Dit on Ring.  Swap probably needed for most paddles.
// Mic should have Mic on Tip, PTT on Ring for this radio.

/*
 * The encoder switch works like this:
 * A tap changes the tuning step with 1k, 100, 10 in rotation.
 *   To change to a larger tuning step, first tap then double tap within 1.5 seconds and select 5k, 50k, 500k steps with double taps.
 * Normally a double tap brings up the multi function control.  When done adjusting, double tap back to normal tuning.
 *   Or single tap for RF gain control ( actually gain in the agc amps ), another tap for CW decoder level, etc.
 * Long Press normally brings up the menu.  You can make selections with a tap.  Double tap exits.  And see sticky menus option below for 
 *   different behavior.  If RIT is active, Long Press cancels RIT.
 *   
 * RIT is automatically enabled upon transmit.  Cancel with a long press to return to normal frequency tuning.  RIT is not tracked in 
 * the usual sense, instead Si5351 PLLB is not updated with new frequency changes when RIT is active. (  A hidden vfo B )
 * 
 * 
 * ***********  OR you can think of the encode switch this way
 * There are 3 actions: tap, double tap, and long press.
 * 
 * The primary use of tap is to cycle through tuning step sizes.
 *    Secondary use of tap is to make selections in the menu and to cycle through multi function selections.
 * The primary use of double tap is to bring up the multi function control and to exit.   
 *    Secondary use of double tap is to increase the tuning step higher than 1k. ( tap then double tap, double tap etc. )
 *    Secondary use of double tap is to escape out of the menu when using the sticky menu complile option.
 * The primary use of long press is to bring up the menu system.   
 *    Seconday use of long press is to turn off RIT.
 */

//  Issues / to do
//  check that the microphone preamp is in a linear region after we added the series resistor in vcc feed. 
//  C4 C7 - does adding them cause processor noise in the front end.  They are currently uninstalled. 
//  Measure audio image response, and alias images at +-44k away.
//  Cut Vusb etch on the Teensy.
//  Cap touch keyer/ptt inputs wired.
//  Dit Dah and PTT seem to be mixed up.  Wiring error or code error?  ( PTT is on Dah, expected it on Dit ).
//  Investigate the filter method used in usdx code for the morse decode signal level. Exponential averaging?
//  Voice transmit with a microphone sounds terrible.  FT8 using USB audio however works great. 
//
//  If a PC is used as a microphone driving the transmitter in USB mode and the PC microphone volume is at max, the program hangs when
//     returning to rx mode.  Seems to be when the AGC is releasing.
//     This is accomplished by using the PC feature "listen to this device" with microphone in and Teensy USB out.
//     This was a transmit test and not how the radio would normally be used.   Note: save this text. 
//
 
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
#define RX      8         //PB0    (pin 14)     4     + PullUp R36 installed
#define SIDETONE 9        //PB1    (pin 15)     A14  DAC wired seperately from SIDETONE net
#define KEY_OUT 10        //PB2    (pin 16)     5     + PullDown  10k ( need PWM pin )
#define SIG_OUT 11        //PB3    (pin 17)     PullDown  10k, no Teensy connection to SIG_OUT net
#define DAH     12        //PB4    (pin 18)     22      on jack it is tip
#define DIT     13        //PB5    (pin 19)     23      on jack it is ring
#define AUDIO1  14        //PC0/A0 (pin 23)     A2
#define AUDIO2  15        //PC1/A1 (pin 24)     A3
#define DVM     16        //PC2/A2 (pin 25)     goes to source of a BS170, drain to A3 ( Q audio2 ), gate switched by 14
#define BUTTONS 17        //PC3/A3 (pin 26)     12     single digital encoder switch,  active low
#define LCD_RS  18        //PC4    (pin 27)     N/C
#define SDA     18        //PC4    (pin 27)     A4
#define SCL     19        //PC5    (pin 28)     A5
//   Nokia Display
        RST                                      8
        CS                                      10
        D/C                                      9
        MOSI                                    11
        CLK                                     13   ?? does switching the LED generate noise, swap with pin 8?

     Capacitance keyer                  pins     0 and 1        
*/
#define RX          4
#define ATTN2       4
#define KEYOUT      5  //20
#define DAHpin     22
#define DITpin     23
#define PTT        23
#define TXAUDIO_EN 14             // switches a FET to put DVM mic signal on Q audio input A3.

//  Pick a screen:  Nokia LCD or I2C 128x64 OLED
//  Running both together yields redefinition warnings as they are basically the same library, but it works.
#define USE_OLED
#define USE_LCD

                                      
#define DEBUG_MP  0                  // !!!  Think will delete this eventually and remove all the if statements it causes in the code
                                     // There is a testing anomally where if the PWM on KEYOUT is enabled and the Teensy powered over
                                     // usb and the radio not powered, the radio gets 4 volts and there is feedback in the rx to mic
                                     // circuits. Will that happen when the radio is powered?  DEBUG_MP set to 1 disables the PWM and
                                     // feedback not present and serial prints active - this is a testing mode. 
                              

// Nokia library uses soft SPI as the D/C pin needs to be held during data transfer.
// 84 x 48 pixels or 6 lines by 14 characters in text mode
#ifdef USE_LCD
  // I bypassed the pin to port code in this copy of the library as it didn't compile for ARM
  // and changed to use digitalWriteFast on the signals and renamed to _t version.
 #include <LCD5110_Basic_t.h>
#endif
#ifdef USE_OLED 
 #include <OLED1306_Basic.h>   // this is the LCD5110 library modified to use I2C for OLED 1306.
#endif

#include <i2c_t3.h>            // non-blocking wire library
#include "MagPhase.h"          // transmitting audio object
#include "AM_decode.h"         // the simplest complex IQ decoder that I tried
#include "my_morse.h"          // my morse table, designed for sending but used also for receive
#include "FFT_Scope.h"         // A simple FFT type of display using a Goertzel filter.



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
#define MULTI_FUN 2
int encoder_user;

// volume users - general use of volume code
#define MAX_VUSERS 7
#define VOLUME_U   0
#define AGC_GAIN_U 1
#define CW_DET_U   2
#define SIDE_VOL_U 3
#define WPM_U      4
#define TONE_U      5
#define TX_DRIVE_U  6
int multi_user;

// screen users of the bottom part not used by freq display and status line
#define INFO 0
#define CW_DECODE 1
#define FFT_SCOPE 2
int screen_user = INFO;  //!!!FFT_SCOPE;  test mic

#define MIC 0
#define USBc 1        // universal serial bus, conflicts with upper side band def USB, so be careful here.  
#define SIDETONE 2 
int tx_source = USBc; // starting on 17 meters FT8 with USBc audio in and out. 

#define DIT 1
#define DAH 2
int key_mask = DIT + DAH;       // feature to disable hardware keyer/PTT if jack is shorted upon power up.

// Menus:  Long press to enter
//   STICKY_MENU 0 :  Tap makes a selection and exits.   Double tap exits without a selection.
//   STICKY_MENU 1 :  Tap makes a selection.  Double tap makes a selection and exits.
// Sticky on is good for making a bunch of menu setups all at once, off is good for making just one change. 
#define STICKY_MENU 0

struct BAND_STACK{
   int mode;
   uint32_t freq;
   int stp;
   int d;             // pre-calculated divider for each band. Can run Si5351 in spec except on 80 meters.
   int tx_src;
   int fltr;
};

// "CW", "LSB", "USB", "AM", "DIGI"
#define CW   0
#define LSB  1
#define USB  2
#define AM   3
#define DIGI 4

struct BAND_STACK bandstack[] = {    // index is the band
  { LSB ,  3928000, 1000, 126, MIC,  4 },
  { USB ,  5330500,  500, 126, MIC,  5 },     // special 500 step for this band, not reachable in the normal step changes.
  { LSB ,  7163000, 1000, 100, MIC,  3 },
  { CW  , 10105000,  100,  68, USBc, 1 },
  { USB , 14100000, 1000,  54, MIC,  3 },
  { DIGI, 18100000, 1000,  40, USBc, 3 }
};

uint32_t freq = 18100000L;     // probably should init these vars from the bandstack
int rit_enabled;
int step_ = 1000;
int band = 5;
int mode = DIGI;
int filter = 4;
int bfo;
int magp;                      // tx drive display on the LCD only
int magpmax;                   // max tx drive display on OLED after return to rx mode.  Can't write to OLED during transmit.
int php;                       // !!! testing phase print
int32_t rav_df;                // digi mode tx filters
int32_t rav_mag;
int trigger_;                  // !!! testing for debug using arduino plotter 

int step_timer;                // allows double tap to backup the freq step to 500k , command times out
                               // and returns to the normal double tap command ( volume )
float af_gain = 0.3;
float agc_gain = 1.0;
float side_gain = 0.1;         // side tone volume, also af_gain affects side tone volume. agc_gain doesn't.
float sig_rms;
int transmitting;
float cw_det_val = 1.3;        // mark space detect, adjust in volume options ( double tap, single tap )
float agc_sig = 0.3;           // made global so can set it after tx and have rx process bring up the af gain
int attn2;                     // attenuator using T/R switch, very large decrease in volume
int wpm = 14;                  // keyer speed, adjust with "Volume" routines
float tone_;                   // tone control, adjust Q of the bandwidth object
float tx_drive = 6.0;          // but probably best to adjust USBc drive with the computer
                               // and use this for the microphone and sidetone testing tx level


#define STRAIGHT    0          // CW keyer modes
#define ULTIMATIC   1
#define MODE_A      2
#define MODE_B      3
#define PRACTICE_T  4          // last options toggle practice and use of the touch keyer, and swap
#define TOUCH_T     5          // put them in this menu as special cases to avoid more yes/no menu's
#define KEY_SWAP    6
int key_mode = ULTIMATIC;
int touch_key = 0;
int cw_practice = 1;
int key_swap = 1;              // jack wired with tip = DAH, needs swap from most of my other radio's

#define stage(c) Serial.write(c)

/******************************** Teensy Audio Library **********************************/ 

#include <Audio.h>
//#include <Wire.h>
//#include <SPI.h>
//#include <SD.h>
//#include <SerialFlash.h>


// Scope added
// GUItool: begin automatically generated code
AudioInputAnalogStereo   adcs1;          //xy=234.99997329711914,284.9999809265137
AudioAnalyzePeak         peak1;          //xy=344.3332977294922,144.3333396911621
AudioInputUSB            usb2;           //xy=344.047607421875,444.61907958984375
AudioAmplifier           agc2;           //xy=389,311.6667175292969
AudioAmplifier           agc1;           //xy=392.8333740234375,261.33331298828125
AudioFilterBiquad        QLow;           //xy=413.83331298828125,360.25
AudioFilterBiquad        ILow;           //xy=429.4999694824219,208.41665649414062
AudioFFT_Scope2          Scope2;         //xy=450,510
AudioMixer4              TxSelect;       //xy=569.3333740234375,445.61907958984375
AudioSynthWaveformSine   sinBFO;         //xy=573.083251953125,307.41668701171875
AudioSynthWaveformSine   cosBFO;         //xy=575.9166259765625,270.75
AudioAnalyzeToneDetect   Scope_det1;          //xy=603.75,510
AudioAnalyzeToneDetect   Scope_det2;          //xy=603.75,510
AudioAnalyzeToneDetect   Scope_det3;          //xy=603.75,510
AudioAnalyzeToneDetect   Scope_det4;          //xy=603.75,510
AudioEffectMultiply      Q_mixer;        //xy=607.9999389648438,363.66668701171875
AudioEffectMultiply      I_mixer;        //xy=610.8333129882812,214.58334350585938
AudioAMdecode2           AMdet;         //xy=620.714183807373,145.71428680419922
AudioSynthWaveformSine   SideTone;       //xy=764.0000343322754,347.95239448547363
AudioMagPhase1           MagPhase;         //xy=779.2856788635254,445.4761657714844
AudioMixer4              SSB;            //xy=780.7618827819824,278.4762382507324
AudioFilterBiquad        BandWidth;          //xy=897.5000610351562,204.52377319335938
AudioAnalyzeRMS          rms1;           //xy=946.6665649414062,149.21426391601562
AudioMixer4              Volume;         //xy=974.7619171142578,278.7619285583496
AudioAmplifier           amp1;           //xy=1060,202
AudioOutputAnalog        dac1;           //xy=1097.190330505371,250.61907577514648
AudioAnalyzeToneDetect   CWdet;          //xy=1104.7025146484375,139.3571014404297
AudioOutputUSB           usb1;           //xy=1118.6191940307617,313.61904525756836
AudioConnection          patchCord1(adcs1, 0, peak1, 0);
AudioConnection          patchCord2(adcs1, 0, agc1, 0);
AudioConnection          patchCord3(adcs1, 0, Scope2, 0);
AudioConnection          patchCord4(adcs1, 1, agc2, 0);
AudioConnection          patchCord5(adcs1, 1, Scope2, 1);
AudioConnection          patchCord6(usb2, 0, TxSelect, 1);
AudioConnection          patchCord7(agc2, QLow);
AudioConnection          patchCord8(agc1, ILow);
AudioConnection          patchCord9(QLow, 0, Q_mixer, 0);
AudioConnection          patchCord10(QLow, 0, TxSelect, 0);
AudioConnection          patchCord11(QLow, 0, AMdet, 1);
AudioConnection          patchCord12(ILow, 0, I_mixer, 0);
AudioConnection          patchCord13(ILow, 0, AMdet, 0);
AudioConnection          patchCord14(Scope2, Scope_det1 );
AudioConnection          patchCord31(Scope2, Scope_det2 );
AudioConnection          patchCord32(Scope2, Scope_det3 );
AudioConnection          patchCord33(Scope2, Scope_det4 );
AudioConnection          patchCord15(TxSelect, 0, MagPhase, 0);
AudioConnection          patchCord16(sinBFO, 0, Q_mixer, 1);
AudioConnection          patchCord17(cosBFO, 0, I_mixer, 1);
AudioConnection          patchCord18(Q_mixer, 0, SSB, 2);
AudioConnection          patchCord19(I_mixer, 0, SSB, 1);
AudioConnection          patchCord20(AMdet, 0, SSB, 0);
AudioConnection          patchCord21(SideTone, 0, Volume, 3);
AudioConnection          patchCord22(SideTone, 0, TxSelect, 2);
AudioConnection          patchCord23(SSB, BandWidth);
AudioConnection          patchCord24(BandWidth, 0, Volume, 0);
AudioConnection          patchCord25(BandWidth, rms1);
AudioConnection          patchCord26(BandWidth, amp1);
AudioConnection          patchCord27(Volume, dac1);
AudioConnection          patchCord28(Volume, 0, usb1, 0);
AudioConnection          patchCord29(Volume, 0, usb1, 1);
AudioConnection          patchCord30(amp1, CWdet);
// GUItool: end automatically generated code


// I2C functions that the OLED library expects to use.
void i2init(){

  Wire.begin(I2C_OP_MODE_DMA);   // use mode DMA or ISR 
  Wire.setClock(800000);     // I2C0_F  40 = 100k ,  25 = 400k.  800000 seems to work, returns 818, 600k returns 600
                             // clock stretching may produce reduced speed if clock is way to fast for the devices.
                             // Calc'd that 700k speed could maybe support a tx bandwidth of 5.5k at 11029 sample rate.
                             // It doesn't.  Nor does 800k or 1000k.
                             // At 1/8 rate:  500k works.  Use 700k for some margin of error.  1/8 of 44117.
                             // At 1/6 rate get some errors at 700k. Use 800k. Should have better TX quality at 1/6 rate.
                             // At 1/5 rate get some errors at 1000k.  Think we should stay at 1/6 rate, ( 1/6 of 44117 )
                             // and 800k clock on I2C.
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


// this include seems to need to be here.  If at top of file with other includes, get a compiler bug error.  

#include "si5351_usdx.cpp"     // the si5351 code from the uSDX project, modified slightly for RIT and dividers used.
SI5351 si5351;                 // maybe it should be done this way.


// the transmit process uses I2C in an interrupt context.  Must prevent other users from writing on I2C.  
// No frequency changes or any OLED writes.  transmitting variable is used to disable large parts of the system.
#define DRATE 6                      // decimation rate used in MagPhase
#define F_SAMP_TX (44117/DRATE)      // ! setting _UA and sample rate the same, removed scaling calculation
#define _UA (44117/DRATE)            // match definition in MagPhase.cpp
#define DLAY 6                       // delay phase changes with respect to amplitude. actual delay is 8 - DLAY in a modulo way. 

int eer_count;
int temp_count;          // !!! debug
int eer_adj;             // !!! debug
int overs;               // I2C not ready for the next set of register data
int saves;               // short write bulk
// float eer_time = 90.680;  //90.668;  // us for each sample deci rate 4
float eer_time = 136.0;  // 1/6 rate ( 1/6 of 44117 )
// float eer_time = 113.335;   // 1/5 rate

void EER_function(){     // EER transmit interrupt function.  Interval timer.
int c;
static int prev_phase;
static int last_dp;
static int dline[8];     // phase change delay
static int din;          // delay index
int phase_;
int mag;
int dp;

   if( MagPhase.available() == 0 ){       // start with 6 ms of buffered data( more now with 1/6 sample rate )
      eer_count = 0;
      return;
   }

   // process Mag and Phase

   mag = MagPhase.mvalue(eer_count);
   if( mode == DIGI ){
       rav_mag = 27853 * rav_mag + 4915 * mag;
       rav_mag >>= 15;
       mag = rav_mag;
   }
   else if( tx_source == MIC ){             // implement tx drive feature for voice mode only
       mag = (float)mag * tx_drive;
   }
   // will start with 10 bits, test if over 1024, write PWM pin for KEY_OUT.
   mag >>= 5;
   mag = constrain(mag,0,1024);
   magp = mag;
   //if( DEBUG_MP != 1 ) analogWrite( KEYOUT, mag );
   analogWrite( KEYOUT, mag );

   // when signal level is low, phase gets very noisy
   phase_ = MagPhase.pvalue(eer_count);
   dp = phase_ - prev_phase;
   prev_phase = phase_;
   php = phase_ ;                        // !!! testing print phase
   
      // removed scaling dp, instead have _UA same as sample rate
      if( dp < -200 ) dp = dp + _UA;                 // allow some audio image to transmit, avoid large positive spikes when not crossing
                                                     // quadrants in arctan ( large negative spike has _UA added )
      // implement the delay line for voice tx, don't break the DIGI modes that are working well
      if( tx_source == MIC ){
         dline[din] = dp;
         dp = dline[ (din + DLAY) & 7 ];
         ++din;
         din &= 7;
      }
                                                     
      if( dp < 3100  /*&& mag > 50 */ ){             // skip sending out of tx bandwidth freq range, suppress positive spikes that get through
         last_dp = dp;                               // print value before changed for sideband and bfo
         if( mode == LSB ) dp = -dp + bfo;           // bfo offset for weaver
         else dp -= bfo;
         if( mode == DIGI ){                         // filter the phase results, good idea or not?
            rav_df = 27853 * rav_df + 4940 * dp;     // r/c time constant recursive filter .85 .15, increased gain from 4915
            rav_df >>= 15;
           // if( DEBUG_MP == 0 ) dp = rav_df;       // see the difference on serial plotter, else use new value
            dp = rav_df;                             // see the difference on scope dummy load when self testing tx.   
         }
       
          si5351.freq_calc_fast(dp);
          if( Wire.done() )                    // crash all:  can't wait for I2C while in ISR
              si5351.SendPLLBRegisterBulk();
              else ++overs;                     //  Out of time, count skipped I2C transactions
      }
      //else last_dp = 0;   // !!! testing   
           

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
      //if( c == 64 ) ;                                      // two blocks delay is the goal
      if( c < 64-8 ) eer_time += 0.0001, ++eer_adj, ++u;     // slow down  64-8 for 1/4 1/6 versions.  76-8 for 1/5 rate.
      if( c > 64+8 ) eer_time -= 0.0001, --eer_adj, ++u;     // speed up    64-8 rates 1/4 and 1/6
     // leak timer toward what we think is correct
      if( eer_time > 136.01 ) eer_time -= 0.00001, ++u;   //  rate 1/6 version.    version at 1/4 rate use 90.67 to 90.68.
      if( eer_time < 135.99 ) eer_time += 0.00001, ++u;
     // if( eer_time > 113.34 ) eer_time -= 0.00001, ++u;     // rate 1/5 version
     // if( eer_time < 113.33 ) eer_time += 0.00001, ++u;     // use 76 +-8 as index to sync
      
      if( u ) EER_timer.update( eer_time);
   }

    if( DEBUG_MP == 1 ){                          // serial writes in an interrupt function
       static int mod;                               // may cause other unintended issues.  Loss of sync maybe. 
       ++mod;
       if(  mod > 0 && mod < 50 /*||  trigger_ */){
          Serial.print( last_dp );  Serial.write(' ');     //  testing, get a small slice of data
  //        Serial.print( rav_df );  Serial.write(' ');
  //        //Serial.print( php ); Serial.write(' ');
          Serial.print( magp );
          Serial.println(); 
       }
       if( mod > 20000 ) mod = 0;
    }

}

void setup() {
   int contrast = 68;

   pinMode( KEYOUT, OUTPUT );
   digitalWriteFast( KEYOUT, LOW );
   //pinMode( RX, OUTPUT );
   //digitalWriteFast( RX, HIGH );
   pinMode( RX, INPUT );                 // let the 10k pullup to 5 volts as opposed to holding at 3.3 with output pin
   pinMode( TXAUDIO_EN, OUTPUT );
   digitalWriteFast( TXAUDIO_EN, LOW );

     // **** this section could be altered as desired for a electret microphone but may work as is.
     // But AVCC would need to be wired to 3.3 volts.
   pinMode( DAHpin, INPUT_PULLUP );         // has a 10k pullup with the microphone circuit
                                            // but I added a jumper connection for ssb
                                            // so this needs to be INPUT_PULLUP when in CW mode and jumper open in CW position.
                                            // !!! it seems I failed to wire AVCC up to the development board, but that is ok as
                                            // I plan to use a dynamic microphone and mic bias is not desired.
   pinMode( DITpin, INPUT_PULLUP );
   
   delay(1);
   if( read_paddles() != 0 ){
        key_mask = 0;                       // disable the hardware keyer and hardware PTT as it has a short ( microphone connected ? )
        pinMode( DAHpin, INPUT );           // remove any current through the dynamic microphone
        pinMode( DITpin, INPUT );
        touch_key = 1;                      // turn on touch keyer/ptt
   }
     // ****
   
   Serial.begin(1200);                   // usb serial, baud rate makes no difference.  Argo V baud rate is 1200.

   pinMode(EN_A,INPUT_PULLUP);
   pinMode(EN_B,INPUT_PULLUP);
   pinMode(EN_SW,INPUT_PULLUP);

   analogWriteResolution(10);
   

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
     //OLD.printNumI( Wire.getClock()/1000,0,ROW4);
   #endif

   freq_display();
   status_display();
   encoder();             // pick up current position

// Audio Library setup
  AudioNoInterrupts();
  AudioMemory(40);
  
  SSB.gain(1,1.0);             // sub for USB
  SSB.gain(2,-1.0);
  SSB.gain(0,0.0);
  SSB.gain(3,0.0);

  set_tx_source();

  set_af_gain(af_gain);
  set_agc_gain(agc_gain);
  
  filter = 3;
  set_bandwidth();
  CWdet.frequency(700,7);       // 600,6  1000,10 etc... aim for 10ms sample times.  Higher tones will be more accurate.(more samples)
  amp1.gain(10.0);              // more signal into the CW detector

  Scope_det1.frequency(300);
  Scope_det2.frequency(300*16);
  Scope_det3.frequency(300*32);
  Scope_det4.frequency(300*48);

  AudioInterrupts();

  if( screen_user == INFO ){
    delay( 2000 );              // let version stay on screen for 2 seconds
    menu_cleanup();             // erase and display again
    info_headers();
  }
  if( screen_user == FFT_SCOPE ) Scope2.setmode( 1 );

}

  // 0.54119610   butterworth Q's two cascade
  // 1.3065630
  //  0.51763809  butterworth Q's 3 cascade
  //  0.70710678
  //  1.9318517

void set_bandwidth( ){
int bw;
int hp;                              // where the hipass cutoff should be
int wv;                              // where we will put the weaver lowpass

   bw = 3300;                        // remove warning uninitialized variable
   switch( filter ){
       case 0: bw = 300; break;      // data duplicated from menu strings.  Could be improved so that
       case 1: bw = 900; break;     // one doesn't need to maintain data in two places if changes are made.
       case 2: bw = 2700; break;
       case 3: bw = 3000; break;
       case 4: bw = 3300; break;
       case 5: bw = 3600; break;
   }
   
   if( mode == CW ){
       wv = 2500;
       hp = ( bw > 1000 ) ? 200 : 700 - bw/2;
       bw = ( bw > 1000 ) ? bw : 700 + bw/2 ;
   }
   else if( mode == AM || mode == DIGI ) wv = 6000, hp = 100;      // for digi mode, set weaver hole at 3k hz
   else hp = 200, wv = 4000;                                       // SSB

   BandWidth.setHighpass(0,hp,0.70710678);
   BandWidth.setLowpass(1,bw,0.51763809 + tone_);
   BandWidth.setLowpass(2,bw,0.70710678 + tone_);
   BandWidth.setLowpass(3,bw,1.9318517 + tone_/2);
   
   set_Weaver_bandwidth(wv); 
}

void set_attn2(){

   if( attn2 == 0 ){
      pinMode( ATTN2, INPUT );        // off RX high with pullup enables to receive normally 
   }
   else{
      pinMode( ATTN2, OUTPUT );
      digitalWriteFast(ATTN2,LOW);                  // large value attenuator
   }  
}

void set_af_gain(float g){

     Volume.gain(0,g);
     Volume.gain(1,0.0);
     Volume.gain(2,0.0);     // unused
     Volume.gain(3,0.0);     // sidetone

}

void set_agc_gain(float g ){

  if( transmitting ) return;   // disable or leave agc active during transmit ? does it work for MIC compression ? 
  AudioNoInterrupts();
    agc1.gain(g);
    agc2.gain(g);
  AudioInterrupts();
}


void set_Weaver_bandwidth(int bandwidth){
  
  bfo = bandwidth/2;                       // weaver audio folding at 1/2 bandwidth

  ILow.setLowpass(0,bfo,0.50979558);       // filters are set to 1/2 the desired audio bandwidth
  ILow.setLowpass(1,bfo,0.60134489);       // with Butterworth Q's for 4 cascade
  ILow.setLowpass(2,bfo,0.89997622);
  ILow.setLowpass(3,bfo,2.5629154);

  QLow.setLowpass(0,bfo,0.50979558);
  QLow.setLowpass(1,bfo,0.60134489);
  QLow.setLowpass(2,bfo,0.89997622);
  QLow.setLowpass(3,bfo,2.5629154);

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
  if( mode == AM ) return;                 // should we bother with AM transmit?  Maybe if radio covered 10 meters. 

  pinMode(RX, OUTPUT );
  digitalWriteFast( RX, LOW );
  set_af_gain(0.0);                        // mute rx
  transmitting = 1;
  si5351.SendRegister(3, 0b11111011);      // Enable clock 2, disable QSD
  if( rit_enabled == 0 ){                  // auto enable rit on transmit, cancel with long press encoder.
     rit_enabled = 1;                      // sort of like vfo B hidden, B = A on transmit. ( pllB, pllA ).
     step_ = 10;                           // make sure we don't tune far far away too quickly
     freq_display();                       // show RIT in display
     status_display();                     // show new step size
  }
  delay(1);                                // delay or wait for I2C done flag
  if( mode == CW ){
    pinMode(KEYOUT,OUTPUT);
    digitalWriteFast( KEYOUT, HIGH );      //  cw practice mode done elsewhere, doesn't call this function
                                           //  sidetone on and gated done elsewhere
  }
  else{
    if( tx_source == MIC ){
       digitalWriteFast( TXAUDIO_EN, HIGH );           // write TXAUDIO_EN if using mic input
       QLow.setHighpass(0,300,0.707);                  //  configure Q filter to pass tx bandwidth if using mic input
       QLow.setLowpass( 1,2800,0.51763809);
       QLow.setLowpass( 2,2800,0.70710678);
       QLow.setLowpass( 3,2800,1.9318517);
       agc2.gain(0.98);

       // configured TX mux probably somewhere else in menu system, for microphone or usb source         
    }
    analogWriteFrequency(KEYOUT,70312.5);  // match 10 bits at 72mhz cpu clock. https://www.pjrc.com/teensy/td_pulse.html
    analogWrite(KEYOUT,0);
    MagPhase.setmode(1);
    EER_timer.begin(EER_function,eer_time);
  }
  tx_status(1);                            // clear row and print headers on LCD only
  Scope2.setmode( 0 );                     // halt RX FFT
}

void rx(){
  // what needs to change to return to rx mode. 

  noInterrupts();
  if( mode == CW ){
                                           // sidetone off done elsewhere
  }
  else{
    EER_timer.end();
    MagPhase.setmode(0);
  }
  pinMode( KEYOUT, OUTPUT );               // either nointerrupts block or this line solved the double tx current on 2nd tx problem.
  digitalWriteFast( KEYOUT, LOW );         // do this after timer end or it will be turned on again 
  interrupts();
  transmitting = 0;
  saves = overs = 0;                       // reset tx status counters
  digitalWriteFast( TXAUDIO_EN, LOW );     // turn FET audio switch off if its on
  si5351.SendRegister(3, 0b11111111);      // disable all clocks
  #ifdef USE_LCD
     LCD.clrRow(0);
     status_display();
  #endif
  set_bandwidth();                         // return Q filter bandwidth if used for tx, or just some redundant processing
  delay(1);                                // let the dust settle
  //pinMode( RX, INPUT );                  // use pullup to 5 volts. unless attn2 is enabled
  set_attn2();                             // use current attn2 setting for RX
  si5351.SendRegister(3, 0b11111100);      // enable rx clocks
  //delay(1);                                // !!! maybe needed for i2c delay to suppress any rx thumps
  set_af_gain( af_gain );                  // unmute rx
  if( mode != DIGI ) agc_sig = 0.2;        // start with 20 over volume setting in agc system
  #ifdef USE_OLED                          // print max mic volume during transmit
    OLD.print(( char * )"Mod ", 128 - 8*6, ROW2);
    OLD.printNumI( magpmax, RIGHT, ROW2, 4, ' ' );
    magpmax = 0;
  #endif
  if( screen_user == INFO ) info_headers();
  if( screen_user == FFT_SCOPE ) Scope2.setmode( 1 );
}


void S_meter( float sig){
int i;
int s;
int j;
char c;

  c = ( attn2 ) ? 'A' : 'S';                 // a visual of the attenuator setting
  s = sig * 100;
  s = constrain(s,1,9);
  #ifdef USE_OLED
   OLD.gotoRowCol(1,0);  OLD.putch(c); OLD.write(0);
  #endif
  #ifdef USE_LCD
   LCD.gotoRowCol(0,84-6*6); LCD.putch(c);  LCD.write(0);
  #endif  

  j = 0x80;
  for( i = 3; i <= 9; ++i ){                 // remove agc floor reading with i = not zero
     #ifdef USE_OLED
      OLD.write(j,2); OLD.write(0);          // bars, space
     #endif
     #ifdef USE_LCD
      LCD.write(j,2); LCD.write(0);
     #endif
     if( i < s ){
        j >>= 1;
        j |= 0x80;
     }
     else j = 0; 
  }
  s = sig * 10; j = 0xff;
  if( s > 4 ) s = 4;
  for( i = 1; i <= 4; ++i ){
     if( i > s ) j = 0;
     #ifdef USE_OLED
      OLD.write(j,2);  OLD.write(0);
     #endif
     #ifdef USE_LCD
      LCD.write(j,2);  LCD.write(0);
     #endif 
  }
}


void loop() {
static uint32_t tm;
static int done2, done3, done4;
int t;  


   t = encoder();
   if( t ){
      if( encoder_user == MENUS ) top_menu(t);
      if( encoder_user == FREQ ){
         qsy( freq + (t * step_ ));
         freq_display();
      }
      if( encoder_user == MULTI_FUN ) multi_adjust(t);    // generic knob routine, tap for other functions
   }

   if( rms1.available() ){                              // agc
        sig_rms = rms1.read();
        agc_process( sig_rms);
        report_info();
   }

   t = millis() - tm;                         // 1ms routines, loop for any missing counts
   if( t > 10 )  t = 1;                       // first time
   if( t > 0 ){
      tm = millis();
      while( t-- ){
         if( step_timer ) --step_timer;       // 1.5 seconds to dtap freq step up to 500k 
         
         int t2 = button_state(0);
         if( t2 > DONE ) button_process(t2);
         
         if( mode == CW && key_mode != STRAIGHT ) keyer();
         // else if( tx_source != USBc ) ptt();   // USB as tx source always key's via CAT control.
         else ptt();                              // usb audio can use ptt ( dit ) to transmit or use CAT.
         
      }
      if( transmitting ) tx_status(0);
   }

   //if( Serial.availableForWrite() > 20 ) radio_control();      // CAT.  Avoid any serial blocking. fails on Teensy, works on UNO.
   radio_control();                                                     // CAT
   if( mode == CW && CWdet.available() ) code_read( CWdet.read() );     // cw decoder using goertzel algorithm object
   if( screen_user == FFT_SCOPE ){
      if( Scope_det2.available() ) scope_plot( Scope_det2.read(), 15 ), done2 = 1;
      if( Scope_det3.available() ) scope_plot( Scope_det3.read(), 30 ), done3 = 1;
      if( Scope_det4.available() ) scope_plot( Scope_det4.read(), 45 ), done4 = 1;
      if( Scope_det1.available() && done2 && done3 && done4 ){
         scope_plot( Scope_det1.read(), 0);
         done2 = done3 = done4 = 0;
      }
   }
   
}

void scope_plot( float val, int off ){
static int mode = 1;
static int pos = 1;
int modep, posp;
int nextp;
uint32_t  dat, dat2;
uint8_t   low,mid,high;
int col;

   if( encoder_user != FREQ ) return;
   if( transmitting ) return;
   
   //Serial.print( val );
   modep = mode;
   posp = pos + off;
   if( off == 0 ){
      if( ++pos > 15 ){
        pos = 1;
        mode = ( mode == 1 )? 2 : 1;
        Scope2.setmode( mode );
      }
   }
   if( off == 0 ) Scope_det1.frequency( 300 * pos, 3 * pos );         // start next freq bin detect. 3* works good but slow
   if( off == 15 ) Scope_det2.frequency( 300 * (pos + off + 1), 3 * (off + pos + 1) );
   if( off == 30 ) Scope_det3.frequency( 300 * (pos + off + 1), 3 * (off + pos + 1) );
   if( off == 45 ) Scope_det4.frequency( 300 * (pos + off + 1), 3 * (off + pos + 1) );
   
   //val = val * val;
   val = val * val * val;
   dat = (uint32_t)(16777215.0 * val);       // expand to 3 bytes
   dat >>= 1;                                // shift out noise floor
   //dat &= 0x7fff;
   //Serial.println( dat );
   //if( dat == 0 ) return;
   dat2 = 0x800000;
   //dat2 = 0;
   while( dat ){                             // sort of log 2
     dat2 >>= 1;
     dat2 |= 0x800000;
     dat >>= 1;
   }

   
   low = dat2 >> 16;
   mid = dat2 >> 8;
   high = dat2;       // & 0xff;

  // Serial.print( high );  Serial.write(' ');
  // Serial.println( low );

   
   #ifdef USE_LCD
     if( posp <= 41 ){
        if( modep == 1 ) col = 40 + posp;
        else col = 41 - posp;

        LCD.gotoRowCol( 5,col );
        LCD.write( low );
        LCD.gotoRowCol( 4,col );
        LCD.write( mid );
        LCD.gotoRowCol( 3,col );
        LCD.write( high );
     }
   #endif
   #ifdef USE_OLED
        if( modep == 1 ) col = 64 + posp;
        else col = 65 - posp;

        OLD.gotoRowCol( 7,col );
        OLD.write( low );
        OLD.gotoRowCol( 6,col );
        OLD.write( mid );
        OLD.gotoRowCol( 5,col );
        OLD.write( high );
   #endif
  
}

// can show info on LCD but not on OLED while transmitting
void tx_status( int clr ){
static int count;
int num;
int i;

#ifdef USE_OLED                // can only show something at start of tx or at the end
   if( clr ){
       OLD.gotoRowCol(1,0);    // make it short as soon the I2C will be very busy
       OLD.putch('T');
   }
#endif

#ifdef USE_LCD
   if( clr ){
       LCD.clrRow(0);
       LCD.print((char *)"Tcpu ",0,ROW0);
       LCD.print((char *)"Ovr ",6*8,ROW0);
   }

   if( ++count < 100 ) return;                // partial second updates
   count = 0;
   num = AudioProcessorUsage();
   num = constrain(num,0,99);
   LCD.printNumI(num,5*6,ROW0,2,' ');
   num = constrain(overs,0,99);
   LCD.printNumI(num,RIGHT,ROW0,2,' ');
   num = map( magp,0,1024,0,14 );
   LCD.gotoRowCol( 5, 0 );
   for( i = 0; i < num; ++i ) LCD.putch('#');
   for( ; i < 14; ++i ) LCD.putch(' ');
   LCD.printNumI(saves,RIGHT,ROW3);
#endif

   if( magp > magpmax ) magpmax = magp;     // print max mag on OLED after transmit.
   
}



void eer_test(){      // !!! tx debug function
static int sec;
static int freq = 700;
float amp;

return;
  // Serial.print(sec);   Serial.write(' ');
  // Serial.print(eer_adj); Serial.write(' ');
  // Serial.print(temp_count); Serial.write(' ');
  // Serial.print( eer_time,5 ); Serial.write(' ');
  // Serial.println( overs );
   if( tx_source != SIDETONE ){
      tx_source = SIDETONE;
      set_tx_source();
   }
   LCD.printNumF(eer_time,5,0,ROW3);

   eer_adj = 0;
   if( sec == 10 ) sec = 0;
   if( sec == 1 ) tx();             // !!! n second transmit test out of N seconds
   if( sec == 5 ) rx();
   ++sec;
  // trigger_ = 1;
  // delay(5);                           // these delays need to be longer than expected to capture edges
   freq = ( sec & 1 ) ? 1200 : 2100;
   amp  = ( sec & 1 ) ? 0.4 : 0.85;
   SideTone.frequency(freq);
   SideTone.amplitude(amp);
  // delay(20);                          // especially this one
  // trigger_ = 0;
   
 //  if ( sec < 12 && sec >= 3 ){
 //   if( freq <= 1000 ) freq -= 50;
 //   else freq -= 200;
 //  }
 //  if( freq < 300 ) freq = 3600;               // think 4000 was aliasing
}

void button_process( int t ){

    if( transmitting ){
         rx();                   // abort tx on any switch press. do we want this or just return OR
         return;                 // maybe allow tap and double tap, not allow menu long press.
    }                            // problem will be the OLED, can't write during transmit, so can't use the menu's.
    
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
           if( step_ <= 1 ) step_ = 1000;
           step_timer = 1500;                  // 1.5 seconds to dtap up past 1000
           status_display();
        }
        else if( encoder_user == MULTI_FUN ){
           if( ++multi_user >= MAX_VUSERS ) multi_user = 0;
           multi_adjust(0);
        }
      break;
      case DTAP:                              // volume is the main function
        if( encoder_user == MENUS ){          // escape menu
           if( STICKY_MENU ) top_menu(2);     // make selection and then
           menu_cleanup();                    // escape from menus
        }
        else if( encoder_user == FREQ && step_timer && step_ < 500000){     // back up freq step to 5k, 50k, 500k
           step_ *= 10;
           if( step_ == 10000 ) step_ = 5000;
           step_timer = 1500;
           status_display();
        }
        // else volume toggle
        else{
            if( encoder_user == FREQ ) encoder_user = MULTI_FUN, multi_adjust(0);
            else{
              multi_adjust(2);
              encoder_user = FREQ, status_display();
            }
        }
      break;   
      case LONGPRESS:
         if( rit_enabled ){
            rit_enabled = 0;
            qsy( freq );            // reset the hidden vfo B to be equal to A.
            freq_display();
         }
         else encoder_user = MENUS, top_menu(0);
      break;
    }
    button_state(DONE);
}

// once just volume, now general use knob function
void multi_adjust( int val ){
const char *msg[] = {"Volume  ","RF gain ","CW det  ","SideTon ", "Key Spd ", "Tone    ", "TXdrive "}; 
float pval; 

   if( val == 0  ){     // first entry, clear status line
   
      #ifdef USE_LCD
        LCD.print(msg[multi_user],0,ROW0);
        LCD.clrRow(0,6*8);
      #endif
      #ifdef USE_OLED
        OLD.print(msg[multi_user],0,ROW2);
        OLD.clrRow(2,6*8);
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

   pval = 99;
   switch( multi_user ){
      case VOLUME_U:
        if( af_gain > 1.0 ) val *= 2;                // bigger steps, crude log pot simulation
        if( af_gain > 2.0 ) val *= 2;
        af_gain = af_gain + ((float) val * 0.05);
        af_gain = constrain(af_gain,0.0,3.0);
        set_af_gain(af_gain);
        pval = af_gain;
      break;
      case AGC_GAIN_U:
        agc_gain += (float)val * 0.1;
        agc_gain = constrain(agc_gain,0.0,4.0);
        set_agc_gain(agc_gain);
        pval = agc_gain;
      break;
      case CW_DET_U:
        cw_det_val += (float)val * 0.05;
        cw_det_val = constrain(cw_det_val,1.0,2.0);
        pval = cw_det_val;
      break;
      case SIDE_VOL_U:
        side_gain += (float)val * 0.02;
        side_gain = constrain(side_gain,0.0,0.4);     // this gets loud easy
        pval = side_gain;
      break;
      case WPM_U:
        wpm += val;
        wpm = constrain(wpm,12,25);     // faster wanted? - change it here
        pval = wpm;
      break;
      case TONE_U:
        tone_ += (float)val * 0.02;
        tone_ = constrain(tone_,-0.4,0.4);
        set_bandwidth();                // tone is changed by changing the Q of the bandwidth filter
        pval = tone_;
      break;
      case TX_DRIVE_U:
        tx_drive += (float)val * 0.1;
        tx_drive = constrain(tx_drive,0.0,8.0);
        set_tx_source();
        pval = tx_drive;
      break;
   }
   
   #ifdef USE_LCD
      LCD.printNumF(pval,2,6*8,ROW0);
   #endif
   #ifdef USE_OLED
      OLD.printNumF(pval,2,6*8,ROW2);   
   #endif
  
}

void set_tx_source(){
int i;      
float drive;


  //drive = ( tx_source == MIC ) ? tx_drive : 0.98;   // control mic gain via volume, sidetone vol via sidetone amplitude, usb via Computer app
  drive = 0.98;                                       // make sure MagPhase object not overloaded, implement increase drive elsewhere.
  for( i = 0; i < 4; ++i ) TxSelect.gain(i,0.0);
  TxSelect.gain(tx_source,drive);
  //if( tx_source == MIC ){                           // (sub) or audible tone mixed in to keep phase calc happy?
  //   SideTone.frequency( 15 );                      // Hilbert doesn't work well below 300 hz
  //   SideTone.amplitude( 0.1 );                     // but this shows some promise to remove noise on tx
  //   TxSelect.gain(SIDETONE,0.2);                   // 0.5
  //}
  
}

void qsy( uint32_t f ){
static int cw_offset = 700;

    // with weaver rx, freq is the display frequency.  vfo and bfo move about with bandwidth changes.
    if( transmitting ) return;                            // can't use I2C for other purposes during transmit
    freq = f;

    switch( mode ){
       case AM:  f += 2500;  break;            // tune AM off frequency to pass the carrier tone.      
       case CW:  f += cw_offset;               // no break
       case LSB: f -= bfo;   break;
       case USB:
       case DIGI: f += bfo;  break;
    }

    si5351.freq( f, 0, 90, bandstack[band].d );
    if( mode == CW && rit_enabled == 0 ){                 // if RIT enabled, leave the TX frequency fixed
         si5351.freq_calc_fast(-cw_offset + bfo);         // else change it
         si5351.SendPLLBRegisterBulk();                   // TX at freq specified.       
    }
    
}

void status_display(){
const char modes[] = "CW LSBUSBAM DIG";
char msg[4];
char msg2[9];
char buf[20];

    if( transmitting ) return;       // avoid OLED writes
    if( mode > 4 ) return;
    strncpy(msg,&modes[3*mode],3);
    if( mode == CW && cw_practice ) msg[2] = 'p';   // practice mode
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
  bandstack[band].tx_src = tx_source;
  bandstack[band].fltr = filter;
  band = to_band;
  //mode = bandstack[band].mode;
  step_ = bandstack[band].stp;
  freq = bandstack[band].freq;
  tx_source = bandstack[band].tx_src;
  set_tx_source();
  filter = bandstack[band].fltr;
  mode_change(bandstack[band].mode);
//  qsy( bandstack[band].freq );  // done in mode_change
//  status_display();            delay until after screen clear  
}

void mode_change( int to_mode ){

  mode = to_mode;
  //qsy( freq );                                     // redundant with weaver rx, to get phasing correct on QSD
  if( mode == CW || mode == LSB ){
     SSB.gain(1,1.0);             // add for LSB
     SSB.gain(2,1.0);
     SSB.gain(0,0.0);
     SSB.gain(3,0.0);
  }
  else if( mode == AM ){
     SSB.gain(1,0.0);  
     SSB.gain(2,0.0);
     SSB.gain(0,1.0);             // AM audio path select
     SSB.gain(3,0.0);    
  }
  else{
     SSB.gain(1,1.0);             // sub for USB
     SSB.gain(2,-1.0);
     SSB.gain(0,0.0);
     SSB.gain(3,0.0);
  }
  //set_af_gain(af_gain);                              // listen to the correct audio path
  set_bandwidth();                                   // bandwidth is mode dependent
  //if( mode == CW ) pinMode(DAHpin, INPUT_PULLUP);    // accomdate the hardware jumper difference when in CW mode. 
  //else pinMode(DAHpin, INPUT );                      // Let 10k pullup work alone. !!! not wired it seems
  delay(1);
  si5351.SendRegister(177, 0xA0);                    // reset PLL's twice, sometimes on wrong sideband or in never never land
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
   if( screen_user == INFO ) info_headers();
}

void info_headers(){    // print the information headers one time

  #ifdef USE_LCD
    LCD.clrRow(5);
    LCD.gotoRowCol(4,0);
    LCD.puts((char *)"cpu ");      // getting tired of ISO C++ forbidden warning
    LCD.gotoRowCol(4,50);
    LCD.puts((char *)"A/D ");
    LCD.gotoRowCol(5,0);
    LCD.puts((char *)"BW ");
    LCD.gotoRowCol(5,50);
    LCD.puts((char *)"TSrc");
  #endif

  #ifdef USE_OLED
    OLD.gotoRowCol(4,0);
    OLD.puts((char *)"cpu ");
    OLD.gotoRowCol(4,88);
    OLD.puts((char *)"A/D ");
    OLD.gotoRowCol(6,0);
    OLD.puts((char *)"BW ");
    OLD.gotoRowCol(6,88);
    OLD.puts((char *)"TSrc");
  #endif
  
}

//  USA: can we transmit here and what mode
//  lower case for CW portions, upper case for phone segments
//  X - out of band, E - Extra, A - Advanced class, G - General
char band_priv( uint32_t f ){
char r = 'X';

   if( band != 1 ) f = f / 1000;             // test 1k steps
   else f = f / 100;                         // 60 meters, test 500hz steps
   switch( band ){                           // is there an easy way to do this.  This seems like the hard way.
       case 0:
          if( f >= 3500 && f <= 4000 ){
             if( f < 3525 ) r = 'e';
             else if( f < 3600 ) r = 'g';
             else if( f < 3700 ) r = 'E';
             else if( f < 3800 ) r = 'A';
             else r = 'G';
          }
       break;
       case 1:
          if( mode == USB ){
             if( f == 53305 || f == 53465 || f == 53570 || f == 53715 || f == 54035 ) r = 'G';
          }
          // !!! can add CW freq's someday
       break;
       case 2:
          if( f >= 7000 && f <= 7300 ){
             if( f < 7025 ) r = 'e';
             else if ( f < 7125 ) r = 'g';
             else if ( f < 7175 ) r = 'A';
             else r = 'G';
          }
       break;
       case 3:
          if( f >= 10100 && f <= 10150 ) r = 'g';
       break;
       case 4:
          if( f >= 14000 && f <= 14350 ){
             if( f < 14025 ) r = 'e';
             else if( f < 14150 ) r = 'g';
             else if( f < 14175 ) r = 'E';
             else if( f < 14225 ) r = 'A';
             else r = 'G';
          }
       break;
       case 5:
          if( f >= 18068 && f <= 18168 ){
             if( f < 18110 ) r = 'g';
             else r = 'G';
          }
       break;
   }

  return r;
}

void freq_display(){
int rem;
char priv[2];

   priv[0] = band_priv( freq );
   priv[1] = 0;
   rem = freq % 1000;
   #ifdef USE_LCD
    LCD.setFont(MediumNumbers);
    LCD.printNumI(freq/1000,0,ROW1,5,'/');       // '/' is a leading space with altered font table
    LCD.setFont(SmallFont);
    LCD.printNumI(rem,62,ROW2,3,'0');
    LCD.print( priv, RIGHT, ROW1 );
    if( rit_enabled ){
       LCD.clrRow( 1, 0, 6*4 );
       LCD.clrRow( 2, 0, 6*4 );
       LCD.print((char *)"RIT",0,ROW2 );
    }
   #endif

   #ifdef USE_OLED
    OLD.setFont(MediumNumbers);
    OLD.printNumI(freq/1000,4*12,ROW0,5,'/');
    OLD.setFont(SmallFont);
    OLD.printNumI(rem,9*12,ROW1,3,'0');
    OLD.print( priv, RIGHT, ROW0 );
    if( rit_enabled ){
       OLD.clrRow( 0, 4*12, 4*12+6*4 );
       OLD.clrRow( 1, 4*12, 4*12+6*4 );
       OLD.print((char *)"RIT",4*12,ROW1 );      
    }
   #endif
}


#define AGC_FLOOR  0.05             //  was 0.035,  0.07 in my QCX_IF program
#define AGC_SLOPE 6
#define AGC_HANG   500              //  hang == ms time
void agc_process( float reading ){
//static float sig = AGC_FLOOR;
static int hang;
float g;
int ch;                            // flag change needed

    ch = 0;
    
    if( reading > agc_sig && reading > AGC_FLOOR ){       // attack
       agc_sig += 0.001,  hang = 0, ch = 1;
    }
    else if( agc_sig > AGC_FLOOR && hang++ > AGC_HANG/3 ){  // decay
       agc_sig -= 0.0001, ch = 1;
    }

    if( ch ){                                         // change needed
      if( encoder_user == FREQ && transmitting == 0 ) S_meter( agc_sig );
      g = agc_sig - AGC_FLOOR;
      g *= AGC_SLOPE;
      g = agc_gain - g;
      if( g <= 0 ) g = 0.1;
      set_agc_gain(g);
    }                                               

}


int encoder(){         /* read encoder, return 1, 0, or -1 */
static int mod;        /* encoder is divided by 4 because it has detents */
static int dir;        /* need same direction as last time, effective debounce */
static int last;       /* save the previous reading */
int new_;              /* this reading */
int b;

   // if( transmitting && encoder_user != MULTI_FUN ) return 0;         // allow adjusting tx drive while transmitting
   if( transmitting ) return 0;                                         // maybe that is not a good idea when using the OLED, program hangs
   
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
      if( sw ) ++press_ , nopress= 0;
      else ++nopress , press_= 0;
      
      /* switch state machine */
         if( st == IDLE_ && press_ >= DBOUNCE ) st = ARM;
         if( st == DONE && nopress >= DBOUNCE ) st = IDLE_;       /* reset state */

         /* double tap and long detect */
         if( st == ARM && nopress >= DBOUNCE/2 )  st = DTDELAY;
         if( st == ARM && press_ >= 8*DBOUNCE )  st = LONGPRESS; 
         if( st == DTDELAY && nopress >= 4*DBOUNCE ) st = TAP;
         if( st == DTDELAY && press_ >= DBOUNCE )   st = DTAP;
          
     return st;        
}

 // check if need a band change before changing frequency from CAT control
void cat_qsy( int32_t f ){
const int32_t band_breaks[6] = { 4500000,6500000,8500000,13000000,15500000,22500000 };
int  i;

   for( i = 0; i < 6; ++i ){
      if( f < band_breaks[i] ) break;
   }
   if( i == 6 ) i = 5;                   // tuned way above 17 meter filter in this radio
   if( i != band ){
      band_change( i );
      status_display();
   }
   qsy( f );
   freq_display();
}


/*****************************************************************************************/
// TenTec Argonaut V CAT emulation

//int un_stage(){    /* send a char on serial */
//char c;

//   if( stg_in == stg_out ) return 0;
//   c = stg_buf[stg_out++];
//   stg_out &= ( STQUESIZE - 1);
//   Serial.write(c);
//   return 1;
//}

#define CMDLEN 20
char command[CMDLEN];
uint8_t vfo = 'A';

void radio_control() {
static int expect_len = 0;
static int len = 0;
static char cmd;

char c;
int done;

    if (Serial.available() == 0) return;
    
    done = 0;
    while( Serial.available() ){
       c = Serial.read();
       command[len] = c;
       if(++len >= CMDLEN ) len= 0;  /* something wrong */
       if( len == 1 ) cmd = c;       /* first char */
       /* sync ok ? */
       if( cmd == '?' || cmd == '*' || cmd == '#' );  /* ok */
       else{
          len= 0;
          return;
       }
       if( len == 2  && cmd == '*' ) expect_len = lookup_len(c);    /* for binary data on the link */       
       if( (expect_len == 0 &&  c == '\r') || (len == expect_len) ){
         done = 1;
         break;   
       }
    }
    
    if( done == 0 ) return;  /* command not complete yet */
        
    if( cmd == '?' ){
      get_cmd();
     // operate_mode = CAT_MODE;            // switch modes on query cat command
     // if( wwvb_quiet < 2 ) ++wwvb_quiet;  // only one CAT command enables wwvb logging, 2nd or more turns it off
     // mode_display();
    }
    if( cmd == '*' )  set_cmd();
    if( cmd == '#' ){
        pnd_cmd(); 
       // if( wwvb_quiet < 2 ) ++wwvb_quiet;  // allow FRAME mode and the serial logging at the same time
    }

 /* prepare for next command */
   len = expect_len= 0;
   stage('G');       /* they are all good commands */
   stage('\r');

}

int lookup_len(char cmd2){     /* just need the length of the command */
int len;

   
   switch(cmd2){     /* get length of argument */
    case 'X': len = 0; break;
    case 'A':
    case 'B': len = 4; break;
    case 'E':
    case 'P':
    case 'M': len = 2; break;
    default:  len = 1; break ;
   }
   
   return len+3;     /* add in *A and cr on the end */
}

void set_cmd(){
char cmd2;
unsigned long val4;

   cmd2 = command[1];
   switch(cmd2){
    case 'X':   stage_str("RADIO START"); stage('\r'); break; 
    case 'O':   /* split */ 
    break;
    case 'A':   // set frequency
    case 'B':
       val4 = get_long();
       cat_qsy(val4);  
    break;
    case 'E':
       if( command[2] == 'V' ) vfo = command[3];
    break;
    case 'W':    /* bandwidth */
    break;
    case 'K':    /* keying speed */
    break;
    case 'T':    /* added tuning rate as a command */
    break;
    case 'M':
       int i = command[2] - '0';          // untangle Argo V modes and my modes
       i &= 3;   i ^= 3;                  // make sure valid;  swap cw and am, swap usb and lsb
       mode_change(i);
    break;       
   }  /* end switch */   
}

void get_cmd(){
char cmd2;
long arg;
int len, i;

   cmd2 = command[1];   
   stage(cmd2);
   switch(cmd2){
    case 'A':     // get frequency
    case 'B': 
      arg = freq;
      stage_long(arg);
    break;
    case 'V':   /* version */
      stage_str("ER 1010-516");
    break;
    case 'W':          /* receive bandwidth */
       stage(30);
    break;
    case 'M':          /* mode. 11 is USB USB  ( 3 is CW ) vfo A, vfo B */
      i = ( mode ^ 3 ) + '0';                            // see if this works, untangle modes differences mine to Argo V
      if( i == 7 ) i = 1;                                // change DIGI to USB
      stage(i); stage(i);
    break;
    case 'O':          /* split */   
       stage(0);
    break;
    case 'P':         /*  passband slider */
       stage_int( 3000 );
    break;
    case 'T':         /* added tuning rate command */
    break;   
    case 'E':         /* vfo mode */
      stage('V');
      stage(vfo);
    break;
    case 'S':         /* signal strength */
       stage(7);
       stage(0);
    break;
    case 'C':      // transmitting status 
       stage(0);
       if( transmitting ) stage(1);
       else stage(0);
    break;
    case 'K':   /* wpm on noise blanker slider */
       stage( 15 - 10 );
    break;   
    default:           /* send zeros for unimplemented commands */
       len= lookup_len(cmd2) - 3;
       while( len-- ) stage(0);  
    break;    
   }
  
   stage('\r');  
}


void stage_str( String st ){
unsigned int i;
char c;

  for( i = 0; i < st.length(); ++i ){
     c = st.charAt( i );
     stage(c);
  }    
}

void stage_long( long val ){
unsigned char c;
   
   c = val >> 24;
   stage(c);
   c = val >> 16;
   stage(c);
   c = val >> 8;
   stage(c);
   c = val;
   stage(c);
}


unsigned long get_long(){
union{
  unsigned long v;
  unsigned char ch[4];
}val;
int i;

  for( i = 0; i < 4; ++i) val.ch[i] = command[5-i]; // or i+2 for other endian
  return val.v;
}

void stage_int( int val ){
unsigned char c;
   c = val >> 8;
   stage(c);
   c = val;
   stage(c);
}

void stage_num( int val ){   /* send number in ascii */
char buf[35];
char c;
int i;

   itoa( val, buf, 10 );
   i= 0;
   while( (c = buf[i++]) ) stage(c);  
}

void pnd_cmd(){
char cmd2;
   
   cmd2 = command[1];
   switch(cmd2){
     case '0':  rx();  break;    // enter rx mode
     case '1':  tx();  break;    // TX
   }

}

/********************* end Argo V CAT ******************************/

int read_paddles(){                    // keyer and/or PTT function
int pdl;
int tch_dit;
int tch_dah;


   pdl = digitalReadFast( DAHpin ) << 1;
   pdl += digitalReadFast( DITpin );

   pdl ^= 3;                                                // make logic positive
   pdl &= key_mask;                                         // apply disable mask ( mono plug microphone is connected on powerup )
   if( key_swap && mode == CW && key_mode != STRAIGHT ){    // only swap CW keyer mode. 
      pdl <<= 1;                                            // don't swap the PTT pin, DITpin, when in SSB mode
      if( pdl & 4 ) pdl += 1;                               // don't swap the CW straight mode pin DITpin
      pdl &= 3;                                             // wire straight key to ring of plug
   }

   // add touch as input option. Any swap will be a physical swap of wires.
   if( touch_key ){
      tch_dit = touchRead( 0 );
      tch_dah = touchRead( 1 );
     // Serial.print(tch_dit); Serial.write(' ');
     // Serial.println(tch_dah);
      if( tch_dit > 700 ) pdl |= DIT;
      if( tch_dah > 700 ) pdl |= DAH;
   }

   return pdl;
}

void side_tone_on(){

  SideTone.frequency(700);
  SideTone.amplitude(side_gain);
  Volume.gain(0,0.0);
  Volume.gain(3,af_gain);   // or make this a constant to remove volume setting from the sidetone adjustment
  if( cw_practice == 0 ) tx();
}

void side_tone_off(){

  Volume.gain(3,0.0);
  if( transmitting ) rx();
  Volume.gain(0,af_gain);
}

void ptt(){                // ssb PTT or straight key, this uses DIT input because DAH is the MIC input
static uint32_t dbounce;
static int txing;          // for cw practice mode to work, duplicate the transmitting variable
int pdl;

   pdl = read_paddles() & DIT;
   dbounce >>= 1;
   if( pdl ) dbounce |= 0x400;    // 8000 is 16ms delay, with 32 bits can double if needed. this will stretch key on time. 

   if( mode == CW ){               // straight key mode
      if( txing && dbounce == 0 ) txing = 0, side_tone_off();
      else if( txing == 0 && dbounce ) txing = 1, side_tone_on();
   }
   else{                           // SSB
//      if( transmitting && dbounce == 0 ) rx();           // version USB audio must use CAT tx control
//      else if( transmitting == 0 && dbounce ) tx(); 
      if( txing && dbounce == 0 ) txing = 0 , rx();        // version USB audio can use CAT or PTT tx control
      else if( txing == 0 && dbounce ) txing = 1, tx(); 
   }
}

// http://cq-cq.eu/DJ5IL_rt007.pdf      all about the history of keyers

#define WEIGHT 200        // extra weight for keyed element

void keyer( ){
static int state;
static int count;
static int cel;           // current element
static int nel;           // next element - memory
static int arm;           // edge triggered memory mask
static int iam;           // level triggered iambic mask
int pdl;

   pdl = read_paddles();
   if( count ) --count;

   switch( state ){
     case 0:                               // idle
        cel = ( nel ) ? nel : pdl;         // get memory or read the paddles
        nel = 0;                           // clear memory
        if( cel == DIT + DAH ) cel = DIT;
        if( cel == 0 ) break;
        iam = (DIT+DAH) ^ cel;
        arm = ( iam ^ pdl ) & iam;         // memory only armed if alternate paddle is not pressed at this time, edge trigger
                                                    // have set up for mode A
        if( key_mode == MODE_B ) arm = iam;         // mode B - the descent into madness begins.  Level triggered memory.
        if( key_mode == ULTIMATIC ) iam = cel;      // ultimatic mode
        
        count = (1200+WEIGHT)/wpm;
        if( cel == DAH ) count *= 3;
        state = 1;
        side_tone_on();
     break; 
     case 1:                                  // timing the current element. look for edge of the other paddle
        if( count ) nel = ( nel ) ? nel : pdl & arm;
        else{
           count = 1200/wpm;
           state = 2;
           side_tone_off();
        }
     break;   
     case 2:                                  // timing the inter-element space
        if( count ) nel = ( nel ) ? nel : pdl & arm;
        else{
           nel = ( nel ) ? nel : pdl & iam;   // sample alternate at end of element and element space
           state = 0;
        }
     break;   
   }
  
}


// ***************   group of functions for a read behind morse decoder    ******************
//   attempts to correct for incorrect code spacing, the most common fault.
int cread_buf[16];
int cread_indx;
int dah_table[8] = { 20,20,20,20,20,20,20,20 };
int dah_in;

// try using both the tone detect and rms level added together, not sure they work well alone.  NOT done for this test

int cw_detect(float av ){
int det;                        // cw mark space detect
static int count;               // mark,space counts
static float rav;               // running average of signals
static int last_good;           // tone returns zero?
int stored;

//static int mod;
//if( ++mod > 100 ){
// LCD.printNumF(av,4,LEFT,ROW3);
// LCD.printNumF(rav,4,RIGHT,ROW3);    // was once printing sig_rms for the am style decoder !!!
// mod = 0;
//}
 
   if( av < 0.0001 ) av = last_good;      // sometimes tone object returns zero
   else last_good = av;
   // av += sig_rms;                      // add in rms amplitude to the tone detector

   det = ( av > cw_det_val * rav ) ? 1 : 0;
   if( det ) av = av/2.0;                 // if we think have signal, don't add in as much to the noise signal level

   rav = 31.0*rav + av;     // try a longer time constant here?  Maybe shorter for faster code and
   rav /= 32.0;             // longer for slower code would work best to avoid noise in between characters.
                            // trade off for fast fading signals being lost with longer constant.
   

   det = cw_denoise( det );
       
   // debug arduino graph values to see signals
  // Serial.print( 10*av ); Serial.write(' '); Serial.print(10*rav); Serial.write(' ');
  // Serial.write(' '); Serial.println(det);

   stored = 0;
   if( det ){                // marking
      if( count > 0 ){
         if( count < 99 ) storecount(count), stored = 1;
         count = 0;
      }
      --count;
   }
   else{                     // spacing
      if( count < 0 ){
        storecount(count), stored = 1;
        count = 0; 
      }
      ++count;
      if( count == 99 ) storecount(count), stored = 1;  // one second no signal
   }
   
   return stored;
}

// slide some bits around to remove 1 reversal of mark space
int cw_denoise( int m ){
static int val;

   if( m ){
      val <<= 1;
      val |= 1;
   }
   else val >>= 1;

   val &= 7;
   if( m ) return val & 4;      // need 3 marks in a row to return true
   else return val & 2;         // 1 extra mark returned when spacing
                                // so min mark count we see is -2 if this works correctly
                                // this shortens mark count by 1 which may be ok as
                                // the tone detect seems to stretch the tone present time
}

// store mark space counts for cw decode
void storecount( int count ){

     cread_buf[cread_indx++] = count;
     cread_indx &= 15;

     if( count < 0 ){      // save dah counts
        count = -count;
        if( count >= 12 ){      // 12 for 10ms per sample, 40 for 3ms?  work up to 25 wpm
          dah_table[dah_in++] = count;
          dah_in &= 7;
        }
     }
}

void shuffle_down( int count ){    /* consume the stored code read counts */
int i;

  for( i= count; i < cread_indx; ++i ){
    cread_buf[i-count] = cread_buf[i];
  }
  
  cread_indx -= count;
  cread_indx &= 15;     // just in case out of sync  
}


int code_read_scan(int slice){  /* find a letter space */
int ls, i;

/* scan for a letter space */
   ls = -1;
   for( i= 0; i < cread_indx; ++i ){
      if( cread_buf[i] > slice ){
        ls = i;
        break;
      }
   }
   return ls;   
}


unsigned char morse_lookup( int ls, int slicer){
unsigned char m_ch, ch;
int i,elcount;

   /* form morse in morse table format */
   m_ch= 0;  elcount= 1;  ch= 0;
   for( i = 0; i <= ls; ++i ){
     if( cread_buf[i] > 0 ) continue;   /* skip the spaces */
     if( cread_buf[i] < -slicer ) m_ch |= 1;
     m_ch <<= 1;
     ++elcount;
   }
   m_ch |= 1;
   /* left align */
   while( elcount++ < 8 ) m_ch <<= 1;

   /* look up in table */
   for( i = 0; i < 47; ++i ){
      if( m_ch == morse[i] ){
        ch = i+',';
        break;
      }
   }

   return ch;  
}

// routines from my TenTec Rebel code
void code_read( float val ){  /* convert the stored mark space counts to a letter on the screen */
int slicer;
int i;
unsigned char m_ch;
int ls,force;
static int wt;    /* heavy weighting will mess up the algorithm, so this compensation factor */
static int singles;
static int farns,ch_count;
static int eees;
static uint32_t tm;

   if( ( millis() - tm ) < 9 ) return;     // run at 10ms rate
   tm = millis();
   if( transmitting) return;
   if( encoder_user != FREQ ) return;
   if( screen_user != CW_DECODE ) return;
   
   if( cw_detect( val ) == 0 && cread_indx < 15 ) return;

   if( cread_indx < 2 ) return;    // need at least one mark and one space in order to decode something

   /* find slicer from dah table */
   slicer= 0;   force= 0;
   for( i = 0; i < 8; ++i ){
     slicer += dah_table[i];
   }
   slicer >>= 4;   /* divide by 8 and take half the value */

   ls = code_read_scan(slicer + wt);
   
   if( ls == -1 && cread_indx == 15 ){   // need to force a decode
      for(i= 1; i < 30; ++i ){
        ls= code_read_scan(slicer + wt - i);
        if( ls >= 0 ) break;
      } 
      --wt;    /* compensate for short letter spaces */
      force= 1;
   }
   
   if( ls == -1 ) return;
   
   m_ch = morse_lookup( ls, slicer );
   
   /* are we getting just E and T */
   if( m_ch == 'E' || m_ch == 'T' ){   /* less weight compensation needed */
      if( ++singles == 4 ){
         ++wt;
         singles = 0;
      }
   }
   else if( m_ch ) singles = 0;   
 
   /* are we getting just e,i,s,h,5 ?   High speed limit reached.  Attempt to receive above 30 wpm */
  // if( m_ch > 0 && ( m_ch == 'S' || m_ch == 'H' || m_ch == 'I' || m_ch == '5' )) ++shi5;
  // else if( m_ch > 0 && m_ch != 'E' ) --shi5;   // E could be just noise so ignore
  // if( shi5 < 0 ) shi5 = 0;
 
   /* if no char match, see if can get a different decode */
   if( m_ch == 0 && force == 0 ){
     //if( ( slicer + wt ) < 10 ) ls = code_read_scan( slicer + wt -1 );
     //else ls = code_read_scan( slicer + wt -2 );
     ls = code_read_scan( slicer + wt - ( slicer >> 2 ) );
     m_ch = morse_lookup( ls, slicer );
     if( m_ch > 64 ) m_ch += 32;       // lower case for this algorithm
     //if( m_ch ) --wt;     this doesn't seem to be a good idea
   }
 
   if( m_ch ){   /* found something so print it */
      ++ch_count;
      if( m_ch == 'E' || m_ch == 'I' ) ++eees;         // just noise ?
      else eees = 0;
      if( eees < 5 ){
         decode_print(m_ch);
         //if( TerminalMode ) Serial.write(m_ch), ++tcount;
      }
      if( cread_buf[ls] > 3*slicer + farns ){   // check for word space
        if( ch_count == 1 ) ++farns;            // single characters, no words printed
        ch_count= 0;
        decode_print(' ');
        //if( TerminalMode ) Serial.write(' '), ++tcount;
        //if( tcount > 55 ) tcount = 0, Serial.println();      // this is here so don't split words
      }
   }
     
   if( ls < 0 ) ls = 0;   // check if something wrong just in case  
   shuffle_down( ls+1 );  

   /* bounds for weight */
   if( wt > slicer ) wt = slicer;
   if( wt < -(slicer >> 1)) wt= -(slicer >> 1);
   
   if( ch_count > 10 ) --farns;
   
}


void decode_print( char c ){

   if( transmitting == 0 && ( encoder_user == FREQ || encoder_user == MULTI_FUN ) ){
      #ifdef USE_LCD
         LCDcwprint( c );
      #endif
      #ifdef USE_OLED
         OLDcwprint( c );
      #endif
   }
}

#ifdef USE_LCD                    // use 3 lines of 14 characters
void LCDcwprint( char c ){
static int row = ROW3, col = 0;
char s[3];                        // no goto yet anyway

   if( row == ROW3 ) c = tolower(c);   // display space tight below bigger numbes
   s[0] = c;  s[1] = ' '; s[2] = 0;    // erase ahead
   if( col == 84 - 6 ) s[1] = 0;       // except on end of line
   LCD.print(s,col,row);
   col += 6;
   if( col > 84 - 6 ) col = 0, row += 8;
   if( row > ROW5 ) row = ROW3;
}
#endif

#ifdef USE_OLED                   // use 4 lines of 21 characters
void OLDcwprint( char c ){
static int row = 4, col = 0;

   if( row == 4 ) row = 5;        // two row, comment these two lines to use 4 rows
   if( row == 6 ) row = 7;        // 4 rows on tiny display is hard to read.
   
   OLD.gotoRowCol( row, col );
   OLD.putch(c);
   col += 6;
   if( col > 126 - 6 ) col = 0, ++row;
   if( row > 7 ) row = 4;
   OLD.putch(' ');
}
#endif

//  ***************   end of morse decode functions





/*****************   MENU's *******************************/
struct MENU {
  const int no_sel;                 // number of selections
  const char title[15];             // top line - 14 char max in title
  const char choice[8][9];          // selections to display - max 8 selections - max text length is 8
};

struct MENU mode_menu = {
   5,
   "Select Mode",
   { "CW", "LSB", "USB", "AM", "DIGI" }          
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

struct MENU tx_source_menu = {
    3,
    "Tx Source",
    {"Mic", "USB", "Side Tn"}
};

struct MENU attn_menu = {
    2,
    "Attenuator",
    { "OFF", "ON" }
};

struct MENU keyer_menu = {
    7,
    "Keyer Mode",
    { "Straight", "UltiMatc", "Mode A", "Mode B", "Practice", "TouchKey", "Swap" }
};

struct MENU screen_menu = {
    3,
    "Decode Screen",
    { "Info", "CW decod", "Scope" }
};

struct MENU main_menu = {
  7,
  "Top Menu",
  { "Band", "Mode", "Filter", "Tx Src", "ATTN", "Keyer", "Decode" }
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
              case 3: active_menu = &tx_source_menu; def_val = tx_source; state = 4; break;
              case 4: active_menu = &attn_menu;  def_val = attn2; state = 5; break;
              case 5: active_menu = &keyer_menu; def_val = key_mode; state = 6; break;
              case 6: active_menu = &screen_menu; def_val = screen_user; state = 7; break;             
            }
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
         case 4:
            tx_source = def_val;
            set_tx_source();
            ret_val = state = 0;
         break;
         case 5:
            attn2 = def_val;
            set_attn2();
            ret_val = state = 0;
         break;
         case 6:
            if( def_val == PRACTICE_T ) cw_practice ^= 1;
            else if( def_val == TOUCH_T ) touch_key ^= 1;
            else if( def_val == KEY_SWAP ) key_swap ^= 1;
            else key_mode = def_val;
            ret_val = state = 0;
         break;
         case 7:
            screen_user = def_val;
            if( screen_user != FFT_SCOPE ) Scope2.setmode( 0 );
            else Scope2.setmode( 1 );                            // may be wrong mode for one pass
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


// put this function here, a hack to pick up the bandwidth menu defines

void report_info(){
static int count;
float val;
char ts[2];

  if( peak1.available() == 0 ) return;
  val = peak1.read();
  val = constrain(val,0.0,0.99);
  
  if( ++count < 333 ) return;            // once a second for printing
  if( DEBUG_MP ) eer_test();             //  tx testing
  count = 0;

  if( encoder_user != FREQ ) return;
  if( screen_user != INFO ) return;
  if( transmitting ) return;

  if( tx_source == MIC ) ts[0] = 'M';
  else if( tx_source == USBc ) ts[0] = 'U';
  else ts[0] = 'S';                               // sidetone
  ts[1] = 0;

  #ifdef USE_LCD
    LCD.printNumI( (int)100*val,RIGHT,ROW4);
    LCD.printNumI( AudioProcessorUsage(),3*6,ROW4,3,' ');   // 0 to 100 percent
    LCD.print(bandwidth_menu.choice[filter],3*6,ROW5);
    LCD.print(ts,RIGHT,ROW5);
  #endif
  #ifdef USE_OLED
    OLD.printNumI( (int)100*val,RIGHT,ROW4);
    OLD.printNumI( AudioProcessorUsage(),3*6,ROW4,3,' ');   // 0 to 100 percent.  More debug
    OLD.print(bandwidth_menu.choice[filter],3*6,ROW6);
    OLD.print(ts,RIGHT,ROW6);
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
 
