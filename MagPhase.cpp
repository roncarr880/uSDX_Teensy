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

// find the magnitude and phase of the transmit audio I and Q streams
// it seems the general idea here is conversion from XY cartesian to polar form
// decimate by N version

#define DRATE 6              // decimation rate.  Input should be lowpass filtered appropriately.

#include <Arduino.h>
#include "MagPhase.h"
#include "utility/dspinst.h"


 static int32_t val1, val2;
 
    //  approx magnitude of I and Q channels
    //  31 * I / 32  + 3 * Q / 8 where I > Q
// estimated async complex square law    
static int32_t fastAM( int32_t i, int32_t q ){ 

    i = abs(i), q = abs(q); 
    if( q > i ){
       q = (31 * q) >> 5;
       i = ( 3 * i) >> 3;
    }
    else{
       i = (31 * i) >> 5;
       q = ( 3 * q) >> 3;
    }
    return ( i + q );
}

static int32_t arctan3( int32_t q, int32_t i ){           // from QCX-SSB code


  // #define _UA  44117/(2*DRATE)
  #define _UA 8448                                  // arbitrary number, common mult of 8 and 22
  #define _atan2(z)  ((_UA/8 - _UA/22 * z + _UA/22) * z)  //derived from (5) [1], see QCX-SSB project for reference quoted
  
  int32_t r;
  int32_t ai, aq;

  ai = abs(i);  aq = abs(q);
  if( aq > ai )   r = _UA / 4 - _atan2( ai / aq );  // arctan(z) = 90-arctan(1/z)
  else r = (i == 0) ? 0 : _atan2( aq / ai );        // arctan(z)
  r = (i < 0) ? _UA / 2 - r : r;                    // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;                          // arctan(-z) = -arctan(z)
}


// a 31 tap classic hilbert every other constant is zero
static void process_hilbert( int16_t val ){
static int32_t wi[31];                                // delay terms
static int32_t wq[31];
const int32_t k0 = (int32_t)( 32767.5 * 0.002972769320862211 );
const int32_t k1 = (int32_t)( 32767.5 * 0.008171666650726522 );
const int32_t k2 = (int32_t)( 32767.5 * 0.017465643081957562 );
const int32_t k3 = (int32_t)( 32767.5 * 0.032878923709314147 );
const int32_t k4 = (int32_t)( 32767.5 * 0.058021930268698417 );
const int32_t k5 = (int32_t)( 32767.5 * 0.101629404192315698 );
const int32_t k6 = (int32_t)( 32767.5 * 0.195583262432201366 );
const int32_t k7 = (int32_t)( 32767.5 * 0.629544595185021816 );

   for( int i = 0; i < 30; ++i )  wi[i] = wi[i+1],  wq[i] = wq[i+1];
   wi[30] = wq[30] = val;

   val2 = wq[15];
   val1 =  k0 * ( wi[0] - wi[30] ) + k1 * ( wi[2] - wi[28] ) + k2 * ( wi[4] - wi[26] ) + k3 * ( wi[6] - wi[24] );
   val1 += k4 * ( wi[8] - wi[22] ) + k5 * ( wi[10] - wi[20]) + k6 * ( wi[12] - wi[18]) + k7 * ( wi[14] - wi[16]);
   val1 >>= 15;

}


void AudioMagPhase1::update(void){ 

    audio_block_t *blk1;
    int16_t *dat1;
    int i;
    static int rem;                  // 128 by 6 has a remainder when done

    // receiving, do nothing
    if( mode == 0 ){
      	blk1 = receiveReadOnly(0);
        if( blk1 ) release( blk1 );
        return;
    }

    // transmitting
    blk1 = receiveReadOnly(0);
    if( blk1 == 0  ){
       //release( blk1 );
       return;
    }
    
    // decimate by DRATE, 6-> sample rate 7353, input must be lowpassed < 3.5k
    dat1 = blk1->data;
    
    for( i = 0; i < AUDIO_BLOCK_SAMPLES; i++ ){

        ++rem;
        if( rem == DRATE ){
           rem = 0;  

           process_hilbert( *dat1 );                           // get val1 and val2            
           mag[count] = fastAM( val1, val2);
           ph[count]  =  arctan3( val1, val2 );
           ++count;    count &=  (AUDIO_BLOCK_SAMPLES-1);     // assume power of two block size ( currently 128 )
        }
        dat1 += 1;                                              
    }
    if( count >= AUDIO_BLOCK_SAMPLES / 2 ) avail = 1;         // have buffered > 6ms of data, avail latches on
    report_count = count;                                     // only report the block ending position of the count
    release( blk1 );
}
