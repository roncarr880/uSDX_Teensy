/* Audio Library for Teensy 3.X
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// One part of an idea for a simple FFT type display created by moving a Goertzel filter around to different frequencies
// and plotting the results.  This audio object runs a simple Hilbert to pick USB or LSB and passes the result on.

#include <Arduino.h>
#include "FFT_Scope.h"
#include "utility/dspinst.h"


// a 31 tap classic hilbert every other constant is zero, has a plus 90 phase shift.
static int16_t SSB( int32_t val1, int32_t val2, int mode ){
static int32_t wi[31];                                // delay terms
static int32_t wq[31];
//const int32_t k0 = (int32_t)( 32767.5 * 0.039466626150415629 );      // no window
//const int32_t k1 = (int32_t)( 32767.5 * 0.045538422602429005 );
//const int32_t k2 = (int32_t)( 32767.5 * 0.053818143717831897 );
//const int32_t k3 = (int32_t)( 32767.5 * 0.065777739272219193 );
//const int32_t k4 = (int32_t)( 32767.5 * 0.084571387356147970 );
//const int32_t k5 = (int32_t)( 32767.5 * 0.118399951005067283 );
//const int32_t k6 = (int32_t)( 32767.5 * 0.197333261348968420 );
//const int32_t k7 = (int32_t)( 32767.5 * 0.591999798557916845 );
const int32_t k0 = (int32_t)( 32767.5 * 0.002972769320862211 );        // with K window
const int32_t k1 = (int32_t)( 32767.5 * 0.008171666650726522 );
const int32_t k2 = (int32_t)( 32767.5 * 0.017465643081957562 );
const int32_t k3 = (int32_t)( 32767.5 * 0.032878923709314147 );
const int32_t k4 = (int32_t)( 32767.5 * 0.058021930268698417 );
const int32_t k5 = (int32_t)( 32767.5 * 0.101629404192315698 );
const int32_t k6 = (int32_t)( 32767.5 * 0.195583262432201366 );
const int32_t k7 = (int32_t)( 32767.5 * 0.629544595185021816 );
int32_t result;

   for( int i = 0; i < 30; ++i )  wi[i] = wi[i+1],  wq[i] = wq[i+1];
   wi[30] = val1;  wq[30] = val2;

   val2 = wq[15];
   val1 =  k0 * ( wi[0] - wi[30] ) + k1 * ( wi[2] - wi[28] ) + k2 * ( wi[4] - wi[26] ) + k3 * ( wi[6] - wi[24] );
   val1 += k4 * ( wi[8] - wi[22] ) + k5 * ( wi[10] - wi[20]) + k6 * ( wi[12] - wi[18]) + k7 * ( wi[14] - wi[16]);
   val1 >>= 15;
   result = ( mode == 1 )? val1 + val2 : val1 - val2;      // pick sideband
   if( result > 32767) result = 32767;
   if( result < -32767 ) result = -32767;
   return (int16_t)( result );

}



void AudioFFT_Scope2::update(void){ 

    audio_block_t *blk1, *blk2;
    int16_t *dat1, *dat2;
    int i;

    // function do nothing 
    if( mode == 0 ){         
     	blk1 = receiveReadOnly(0);
      if( blk1 ){
         // transmit( blk1 );
          release( blk1 );
      }
      blk2 = receiveReadOnly(1);
          if( blk2 ) release( blk2 );
          return;
      }

    blk1 = receiveWritable(0);
    blk2 = receiveReadOnly(1);
    if( blk1 == 0 || blk2 == 0 ){
       if( blk1 ) release( blk1 );
       if( blk2 ) release( blk2 );
       return;
    }
    
    dat1 = blk1->data;
    dat2 = blk2->data;
    for( i = 0; i < AUDIO_BLOCK_SAMPLES; i++ ){
       *dat1++ = SSB( (int32_t)*dat1, (int32_t)*dat2++, mode );                                              
    }
    transmit( blk1 );
    release( blk1 );
    release( blk2 );
}
