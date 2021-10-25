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
// decimate by 6 version, or 5, or N

#define DRATE 6

#include <Arduino.h>
#include "MagPhase.h"
#include "utility/dspinst.h"



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

  //#define _UA  44117/12                                   // just guessing here, half of our sample rate
  #define _UA  44117/2*DRATE
  #define _atan2(z)  ((_UA/8 - _UA/22 * z + _UA/22) * z)  //derived from (5) [1], see QCX-SSB project for reference quoted
  
  int32_t r;
  int32_t ai, aq;

  ai = abs(i);  aq = abs(q);
  if( aq > ai )   r = _UA / 4 - _atan2( ai / aq );  // arctan(z) = 90-arctan(1/z)
  else r = (i == 0) ? 0 : _atan2( aq / ai );        // arctan(z)
  r = (i < 0) ? _UA / 2 - r : r;                    // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;                          // arctan(-z) = -arctan(z)
}


void AudioMagPhase2::update(void){ 

    audio_block_t *blk1, *blk2;
    int32_t val1, val2;
    int16_t *dat1, *dat2;
    int i;
    static int rem;                  // 128 by 6 has a remainder when done

    // receiving, do nothing
    if( mode == 0 ){
      	blk1 = receiveReadOnly(0);
        if( blk1 ) release( blk1 );
        blk2 = receiveReadOnly(1);
        if( blk2 ) release( blk2 );
        return;
    }

    // transmitting
    blk1 = receiveReadOnly(0);
    blk2 = receiveReadOnly(1);
    if( blk1 == 0 || blk2 == 0 ){
       if( blk1 ) release( blk1 );
       if( blk2 ) release( blk2 );
       return;
    }
    
    // decimate by DRATE, 6-> sample rate 7353, input must be lowpassed < 3.5k
    dat1 = blk1->data;
    dat2 = blk2->data;
    for( i = 0; i < AUDIO_BLOCK_SAMPLES; i++ ){

        ++rem;
        if( rem == DRATE ){
           rem = 0;  
        
           //val1 = *dat1 >> 1;   val2 = *dat2 >> 1;           // scale to avoid overflow in square values added
           val1 = (int32_t) *dat1;   val2 = (int32_t) *dat2;      // cast just to show what is happening, use native 32 bit int            
           mag[count] = fastAM( val1, val2);
           ph[count]  =  arctan3( val2, val1 );
           ++count;    count &=  (AUDIO_BLOCK_SAMPLES-1);     // assume power of two block size ( currently 128 )
        }
        dat1 += 1; dat2 += 1;                                              
    }
    if( count >= AUDIO_BLOCK_SAMPLES / 2 ) avail = 1;         // have buffered > 6ms of data, avail latches on
    report_count = count;                                     // only report the block ending position of the count
    release( blk1 );
    release( blk2 );
}
