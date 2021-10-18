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

#include <Arduino.h>
#include "AM_decoder.h"
#include "utility/dspinst.h"


// found on internet somewhere
static unsigned short isqrt(unsigned long a) {
    unsigned long rem = 0;
    unsigned int root = 0;
    int i;

    for (i = 0; i < 16; i++) {
        root <<= 1;
        rem <<= 2;
        rem += a >> 30;
        a <<= 2;

        if (root < rem) {
            root++;
            rem -= root;
            root++;
        }
    }

    return (unsigned short) (root >> 1);
}


    //  approx magnitude of I and Q channels
    //  31 * I / 32  + 3 * Q / 8 where I > Q
static int16_t fastAM( int32_t i, int32_t q ){     // the faster algorithm, carrier perhaps not completely cancelled

    i = abs(i), q = abs(q); 
    if( q > i ){
       int32_t t = i;
       i = q;
       q = t;
    }
    i = (31 * i) >> 5;
    q = ( 3 * q) >> 3;
    return (int16_t)( i + q );

}

    //  isqrt ( i^2 + q^2 )
static int16_t slowAM( int32_t i, int32_t q ){    // the slower algorithm ( only 2% more cpu than the fast algorithm )
                                                  // +5 or +7 from non-am modes
   // i >>= 1;  q >>= 1;
    i = (i * i);
    q = (q * q);
    return (int16_t)( isqrt( i + q ) );

}


void AudioAMdecode2::update(void){     // results need to be lowpass filtered as effective sample rate is 11k
                                       // results need to be hipass filtered to remove DC offset

    audio_block_t *blk1, *blk2;
    int32_t val1, val2;
    int16_t *dat1, *dat2;
    int i, mod;

    if( mode == 0 ){
      	blk1 = receiveReadOnly(0);
	      if( blk1 ) release( blk1 );
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
    
    mod = 0;                // decimate by 4, sample rate 11k, input must be lowpassed < 5k
    dat1 = blk1->data;
    dat2 = blk2->data;
    for( i = 0; i < AUDIO_BLOCK_SAMPLES; ++i ){
        mod = (mod + 1) & 3;
        if( mod != 1 ) *dat1 = 0;
        else{
           val1 = *dat1 >> 1;   val2 = *dat2 >> 1;           // scale to avoid overflow in square values added
           //val1 = *dat1;   val2 = *dat2;                   // result is always positive, so lost half of dynamic range
           if( mode == 1 ) *dat1 = fastAM( val1, val2);
           else *dat1 = slowAM( val1, val2 );
        }
        ++dat1, ++dat2;        
    }
    transmit( blk1 );
    release( blk1 );
    release( blk2 );
}
