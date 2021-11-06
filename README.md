# uSDX_Teensy
An amateur radio project using the uSDX hardware and controlled with a Teensy 3.2.
This project uses the Teensy Audio Library for a large part of the DSP processing and the prototype was built using the QRP-LABS QCX+ transceiver kit.

The concept and a part of the code is based upon the original work of PE1NNZ and can be found here:
https://github.com/threeme3/QCX-SSB

## The Concept
The uSDX is a minimilist hardware design where the software does as much of the work as possible.  The key part is the EER ( envelope elimination and restoration ) transmitter.  The transmit audio is split into I and Q audio, the amplitude is sent to a PWM modulator of the final amplifier and the frequency part is sent to the Si5351.  The transmitter chain does not need to be linear.

## Teensy 3.2
The Teensy 3.2 was chosen for its 5 volt tolerant inputs and the Teensy Audio Library.  It should be noted that PJRC ( makers of Teensy ) offer an Audio shield that would provide superior audio to this minimal hardware design.  The codec contains a gain controlled analog front end and anti-aliasing filters.



