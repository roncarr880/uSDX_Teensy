

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


#include "Arduino.h"

extern void i2start( unsigned char c );
extern void i2send( unsigned int data );
extern void i2stop( );
extern int rit_enabled;
extern int saves;

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
  static uint8_t last_reg3;

    if( last_reg3 == pll_regs[3] ){
       ++saves;
       i2start( SI5351_ADDR );         //i2c.start();
       i2send( 26+1*8 + 4 );           //i2c.SendByte(26+1*8 + 3);  // Write to PLLB
       i2send( pll_regs[4]);           //i2c.SendByte(pll_regs[4]);
       i2send( pll_regs[5]);           //i2c.SendByte(pll_regs[5]);
       i2send( pll_regs[6]);           //i2c.SendByte(pll_regs[6]);
       i2send( pll_regs[7]);           //i2c.SendByte(pll_regs[7]);
       i2stop();                       //i2c.stop();
    }
    else{
       i2start( SI5351_ADDR );         //i2c.start();
                                       //i2c.SendByte(SI5351_ADDR << 1);
       i2send( 26+1*8 + 3 );           //i2c.SendByte(26+1*8 + 3);  // Write to PLLB
       i2send( ( last_reg3 = pll_regs[3] ));           //i2c.SendByte(pll_regs[3]);
       i2send( pll_regs[4]);           //i2c.SendByte(pll_regs[4]);
       i2send( pll_regs[5]);           //i2c.SendByte(pll_regs[5]);
       i2send( pll_regs[6]);           //i2c.SendByte(pll_regs[6]);
       i2send( pll_regs[7]);           //i2c.SendByte(pll_regs[7]);
       i2stop();                       //i2c.stop();
    }
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
  
  void freq(uint32_t fout, uint8_t i, uint8_t q, uint16_t d ){  // Set a CLK0,1 to fout Hz with phase i, q
      uint8_t msa; uint32_t msb, msc, msp1, msp2, msp3p2;
      uint8_t rdiv = 0;             // CLK pin sees fout/(2^rdiv)
      
      //if(fout < 500000){ rdiv = 7; fout *= 128; }; // Divide by 128 for fout 4..500kHz
      //uint16_t d = (16 * fxtal) / fout;  // Integer part
      //if(fout > 30000000) d = (34 * fxtal) / fout; // when fvco is getting too low (400 MHz)     
      //uint16_t d = bandstack[band].d;
      
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
      if( rit_enabled == 0 ) SendRegister(26+1*8, pll_regs, 8); // Write to PLLB unless tx freq is fixed.

      msa = fvcoa / fout;     // Integer part of vco/fout
      msp1 = (128*msa - 512) | (((uint32_t)rdiv)<<20);     // msp1 and msp2=0, msp3=1, not fractional
      uint8_t ms_regs[8] = {0, 1, BB2(msp1), BB1(msp1), BB0(msp1), 0, 0, 0};
      SendRegister(42+0*8, ms_regs, 8); // Write to MS0
      SendRegister(42+1*8, ms_regs, 8); // Write to MS1
      if( rit_enabled == 0 ) SendRegister(42+2*8, ms_regs, 8); // Write to MS2
      SendRegister(16+0, 0x0C|3|0x00);  // CLK0: 0x0C=PLLA local msynth; 3=8mA; 0x40=integer division; bit7:6=0->power-up
      SendRegister(16+1, 0x0C|3|0x00);  // CLK1: 0x0C=PLLA local msynth; 3=8mA; 0x40=integer division; bit7:6=0->power-up
      SendRegister(16+2, 0x2C|3|0x00);  // CLK2: 0x2C=PLLB local msynth; 3=8mA; 0x40=integer division; bit7:6=0->power-up
      SendRegister(165, i * msa / 90);  // CLK0: I-phase (on change -> Reset PLL)
      SendRegister(166, q * msa / 90);  // CLK1: Q-phase (on change -> Reset PLL)
      if(iqmsa != ((i-q)*msa/90)  && rit_enabled == 0 ){
        iqmsa = (i-q)*msa/90; SendRegister(177, 0xA0);
        } // 0x20 reset PLLA; 0x80 reset PLLB
      SendRegister(3, 0b11111100);      // Enable/disable clock

  //Serial.print( fout/1000 ); Serial.write(' ');        // !!! debug
  //Serial.print( fvcoa/1000 ); Serial.write(' ');
  //Serial.print( iqmsa );  Serial.write(' ');
  //Serial.println(d);
     if( rit_enabled == 0 ){          // save tx freq variables if tx frequency changed
        _fout = fout;  // cache
        _div = d;
        _msa128min512 = fvcoa / fxtal * 128 - 512;
        _msb128=((uint64_t)(fvcoa % fxtal)*_MSC*128) / fxtal;
     }
      
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

// static SI5351 si5351;
