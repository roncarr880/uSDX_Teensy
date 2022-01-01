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

// Async complex AM detector
// https://www.dsprelated.com/showarticle/938.php
// Figure 4.  Hilbert and delay not needed as we already have I and Q signals.

#include <Arduino.h>
#include "AM_decode.h"
#include "utility/dspinst.h"

void AudioAMdecode2::update(void){
                                       // results need to be IIR hipass filtered to remove DC offset

    audio_block_t *blk1, *blk2;
    int32_t val1, val2;
    int16_t *dat1, *dat2;
    int i;

 
    blk1 = receiveWritable(0);
    blk2 = receiveReadOnly(1);
    if( blk1 == 0 || blk2 == 0 ){
       if( blk1 ) release( blk1 );
       if( blk2 ) release( blk2 );
       return;
    }
    
    dat1 = blk1->data;
    dat2 = blk2->data;
    for( i = 0; i < AUDIO_BLOCK_SAMPLES; ++i ){    
        val1 = *dat1 >> 1;   val2 = *dat2++ >> 1;           // scale to avoid overflow in square values added
        val1 = abs(val1);    val2 = abs(val2);
        *dat1++ = (int16_t)( val1 + val2 );       
    }
    transmit( blk1 );
    release( blk1 );
    release( blk2 );
}
