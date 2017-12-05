"# TPI_Programmer" 

Project is based on http://junkplusarduino.blogspot.jp/p/attiny10-resources.html. I tried to find a way to contact them for my updates, but could not find their contact info.

TPI programmer for ATtiny4/5/9/10/20/40

Make the connections as shown here http://junkplusarduino.blogspot.jp/p/attiny10-resources.html.

To use:

***** Buad rate must be set to 9600 ****
- Upload to arduino and power off
- Connect ATtiny10 as shown
- Power on and open the serial monitor
- If things are working so far you should
  see "NVM enabled" and "ATtiny10/20/40 connected".
- Input one-letter commands via serial monitor:

D = dump memory. Displays all current memory on the chip

E = erase chip. Erases all program memory automatically done at time of programming

P = write program. After sending this, paste the program from the hex file into the serial monitor.

S = set fuse. follow the instructions to set one of the three fuses.

C = clear fuse. follow the instructions to clear one of the three fuses.

H = Toggle High Voltage Programming  

T = Toggle +12v enabled by High, or Low

R/r = Quick reset

- Finally, power off the arduino and remove the Attiny10/20/40
                                                                                      
thanks to pcm1723 for tpitest.pde upon which this is based                                 
