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

// input must be low pass filtered.
// decimate by 4, process either the amplitude of I and Q for AM, or Hilbert and sum for SSB
// output is upsampled by 4 and must be followed by a lowpass filter 

#define DRATE 4              // decimation rate.  Input should be lowpass filtered appropriately.

#include <Arduino.h>
#include "SSB_AM.h"
#include "utility/dspinst.h"



    //  approx magnitude of I and Q channels
    //  31 * I / 32  + 3 * Q / 8 where I > Q
// estimated async complex square law    
static int16_t fastAM( int32_t i, int32_t q ){ 

    i = abs(i), q = abs(q); 
    if( q > i ){
       q = (31 * q) >> 5;
       i = ( 3 * i) >> 3;
    }
    else{
       i = (31 * i) >> 5;
       q = ( 3 * q) >> 3;
    }
   int32_t result = i + q;
   if( result > 32767) result = 32767;
   if( result < -32767 ) result = -32767;

    return (int16_t)( result);
}


// a 31 tap classic hilbert every other constant is zero, no window
static int16_t SSB( int32_t val1, int32_t val2 ){
static int32_t wi[31];                                // delay terms
static int32_t wq[31];
const int32_t k0 = (int32_t)( 32767.5 * 0.039466626150415629 );
const int32_t k1 = (int32_t)( 32767.5 * 0.045538422602429005 );
const int32_t k2 = (int32_t)( 32767.5 * 0.053818143717831897 );
const int32_t k3 = (int32_t)( 32767.5 * 0.065777739272219193 );
const int32_t k4 = (int32_t)( 32767.5 * 0.084571387356147970 );
const int32_t k5 = (int32_t)( 32767.5 * 0.118399951005067283 );
const int32_t k6 = (int32_t)( 32767.5 * 0.197333261348968420 );
const int32_t k7 = (int32_t)( 32767.5 * 0.591999798557916845 );

   for( int i = 0; i < 30; ++i )  wi[i] = wi[i+1],  wq[i] = wq[i+1];
   wi[30] = val1;  wq[30] = val2;

   val2 = wq[15];
   val1 =  k0 * ( wi[0] - wi[30] ) + k1 * ( wi[2] - wi[28] ) + k2 * ( wi[4] - wi[26] ) + k3 * ( wi[6] - wi[24] );
   val1 += k4 * ( wi[8] - wi[22] ) + k5 * ( wi[10] - wi[20]) + k6 * ( wi[12] - wi[18]) + k7 * ( wi[14] - wi[16]);
   val1 >>= 15;
   int32_t result = val1 + val2;
   if( result > 32767) result = 32767;
   if( result < -32767 ) result = -32767;
   return (int16_t)( result );

}



void AudioSSB_AM2::update(void){ 

    audio_block_t *blk1, *blk2;
    //int32_t val1, val2;
    int16_t *dat1, *dat2;
    int i;
    static int rem;                  // 128 by 6 has a remainder when done

    // function do nothing  - think AM mode will take the place of this to run TX agc
 //   if( mode == 2 ){         
 //     	blk1 = receiveReadOnly(0);
 //      if( blk1 ){
 //           transmit( blk1 );
 //           release( blk1 );
 //       }
 //       blk2 = receiveReadOnly(1);
 //       if( blk2 ) release( blk2 );
 //       return;
 //   }

    // running either AM or SSB
    blk1 = receiveWritable(0);
    blk2 = receiveReadOnly(1);
    if( blk1 == 0 || blk2 == 0 ){
       if( blk1 ) release( blk1 );
       if( blk2 ) release( blk2 );
       return;
    }
    
    // decimate by DRATE, 4 -> sample rate 11k, input must be lowpassed null at 5.5k
    dat1 = blk1->data;
    dat2 = blk2->data;
    for( i = 0; i < AUDIO_BLOCK_SAMPLES; i++ ){

        ++rem;
        if( rem == DRATE ){
           rem = 0;               
           if( mode == 1 ) *dat1 = fastAM( (int32_t)*dat1, (int32_t)*dat2);
           else *dat1 = SSB( (int32_t)*dat1, (int32_t)*dat2 );
        }
        else *dat1 = 0;
        dat1 += 1; dat2 += 1;                                              
    }
    transmit( blk1 );
    release( blk1 );
    release( blk2 );
}
