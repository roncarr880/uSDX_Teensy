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

#ifndef MG_phase_h_
#define MG_phase_h_

#include "Arduino.h"
#include "AudioStream.h"

class AudioMagPhase2 : public AudioStream
{

public:
	AudioMagPhase2(void) : AudioStream(2, inputQueueArray) {

	}
	
	virtual void update(void);
  
  void setmode( int m ){
    mode = m;
    report_count = count = avail = 0;                  // reset all on mode change
  }
 
  int available(){
    return avail; 
  }

  int32_t mvalue( int i ){
    return mag[i];
  }

  int32_t pvalue( int i ){
    return ph[i];
  }

  int read_count(){
    return report_count;
  }
  
private:
  int mode;
  audio_block_t *inputQueueArray[2];
  int mag[AUDIO_BLOCK_SAMPLES];         // 4 blocks of out samples 12ms long
  int  ph[AUDIO_BLOCK_SAMPLES];         // at decimation rate of 4
  int count;                            // data in index
  int report_count;                     // report the block ending count
  int avail;                            // becomes true when we have buffered 6ms of data
};


#endif
