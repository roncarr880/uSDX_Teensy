# uSDX_Teensy
An amateur radio project using the uSDX hardware and controlled with a Teensy 3.2.
This project uses the Teensy Audio Library for a large part of the DSP processing and the prototype was built using the QRP-LABS QCX+ transceiver kit.

The concept and a part of the code is based upon the original work of PE1NNZ and can be found here:
https://github.com/threeme3/QCX-SSB

There is a thread on groupsio for this project:https://groups.io/g/ucx/topic/alternative_processor_project/85821954

![operating](https://github.com/roncarr880/uSDX_Teensy/blob/main/usdx_bandscope.png)


### The Concept
The uSDX is a minimilist hardware design where the software does as much of the work as possible.  The key part is the EER ( envelope elimination and restoration ) transmitter.  The transmit audio is split into I and Q audio, the amplitude is sent to a PWM modulator of the final amplifier and the frequency part is sent to the Si5351.  The transmitter chain does not need to be linear yet can still transmit SSB.

### Teensy 3.2
The Teensy 3.2 was chosen for its 5 volt tolerant inputs and the Teensy Audio Library.  (It should be noted that PJRC ( makers of Teensy ) offer an Audio shield that would provide superior audio to this minimal hardware design.  The codec contains a gain controlled analog front end and anti-aliasing filters.)
The Teensy audio library operates at 44khz and magically streams audio between objects.  It runs on a low priority interrupt and processes blocks of audio that are 128 samples long ( about 3ms long ).  The use of the audio library has some advantages and disadvantages. With one or two hours of work, one can completely redesign the whole radio which I have done many times.  One disadvantage is that the sampling of audio with the two A/D converters completely tie up the A/D hardware and analogRead commands can not be used ( using the audio shield does not have this issue ).  One advantage is that the Teensy can appear in Windows/Unix as a USB soundcard and adding this feature is trivial, a setting in Arduino to compile it in.  

#### The current design.

![design](https://github.com/roncarr880/uSDX_Teensy/blob/main/Design.png)

The receiver uses the Weaver method of decoding SSB.  The vfo is not placed at zero beat with the desired signal but instead is placed right in the middle of the signal.  The I and Q audio is low pass filtered at 1/2 of the desired audio bandwidth.  It is then mixed with a complex BFO also at 1/2 the bandwidth in frequency ( if bandwidth is 3000 hz then the BFO is at 1500 hz ).  The result is added or subtracted to get LSB or USB.

The transmitter re-uses a couple of the receiver audio blocks for the microphone input. The microphone, usb, or the sidetone can be selected as the transmit audio source, and the custom object MagPhase processes the audio and calculates the EER data for the Si5351 and PWM modulator.

#### Wiring

See the marked up high res QCX+ schematic.  You can get the Teensy connections from the sketch.  Most of the Teensy connections are flexible, except for the DAC pin, A2, A3.  The KeyOut pin needs to be PWM and the touch keyer needs touch pins.  The Nokia display if used is using soft SPI and can be wired however one wishes. The OLED if used is connected to the I2C pins ( A4 A5 ).  A4 and A5 also connect to the Si5351.
I did not install C4 C7 as adding them caused processor noise to enter the receiver on a previous project.

![wire1](https://github.com/roncarr880/uSDX_Teensy/blob/main/Side1.png) 
![wire2](https://github.com/roncarr880/uSDX_Teensy/blob/main/Side2.png)



##### Microphone pre-amp.

A jack was installed at the CAT position and wired to a microphone preamplifier circuit installed in the footprint of IC10. The audio is placed on the Teensy A3 pin during transmit via a FET audio switch.  The gate is wired to a Teensy pin.
![wire4](https://github.com/roncarr880/uSDX_Teensy/blob/main/usdx_mic.png)
