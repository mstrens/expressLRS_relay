# ExpressLRS Relay
A common trope for long-range pilots in the search for great performance and longer flights is to put the TX modules up in a pole or to use antenna trackers in order to increase the range by making the signals go over obstacles easier or by using high-gain antennas that has to point to the aircraft to be effective. Commercial systems like the Dragonlink, allow you to do that by providing standard SBUS and Mavilink protocols which can be retransmitted wirelessly to the radio using a wireless usually by a regular RC receiver and/or wifi-fi, Bluetooth allowing the user to stay out of the sun, use larger monitors for FPV, etc.
The technology used in the ExpressLRS TX modules gives us an enormous range and excellent penetration but, due to the characteristics of the protocol (CRFS), the TX module has to be physically attached to the radio, negating the benefits expressed above.
The objective of this project is to close this gap by creating a bidirectional converter for the CRFS protocol used in ExpressLSR and TBS Crossfire. 
Using two cheap microcontrollers boards, the relay will convert the CRFS protocol used to SBUS+SPORT and back allowing the mounting of the ExpresssLRS TX into a pole or tracker, greatly extending its range and mobility.
Currently, only RP2040-zero (and potentially Raspberry Pi Pico) is supported but other platforms with 3 serial ports can be used and may be supported in the future.
### Important: at this point the code is not ready for flying, only bench tests have being done!
### The authors can't take responsibility, personal or otherwise, by the use of this software. It is provided "as is".
This software is licenced under the GPLv3. Please check the LICENSE file for more information but, in short:
#### GPL v3 Requirements
Include a copy of the full license text
State all significant changes made to the original software
Make available the original source code when you distribute any binaries based on the licensed work
Include a copy of the original copyright notice
In addition, GPL v3 states that anyone who includes the code as part of a consumer device has to include any installation information necessary to update and reinstall the software.
#### The GPL v3 license permits users of the code to:
Use the code for commercial purposes: Like GPL v2, GPL v3 imposes no conditions on the internal use of the software.
Change the code: Users can change or rework the code, but if they distribute these changes/modifications in binary form, they’re also required to release these updates in source code form under the GPL v3 license.
Distribute copies or modifications of the code: As long as these modifications are also released under the GPL v3 license, they can be distributed to others.
Place warranty: Distributors of the original code can offer their own warranty on the licensed software.
