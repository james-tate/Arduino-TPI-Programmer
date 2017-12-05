"# TPI_Programmer" 

TPI programmer for ATtiny4/5/9/10/20/40

Make the connections as shown below.

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

Arduino                 ATtiny10              
----------+          +----------------        
(SS#)  10 |--[R]-----| 6 (RESET#/PB3)       
          |          |                 
(MOSI) 11 |--[R]--+--| 1 (TPIDATA/PB0)        
          |       |  |                        
(MISO) 12 |--[R]--+  |                        
          |          |                        
(SCK)  13 |--[R]-----| 3 (TPICLK/PB1)      
    	  |          |
(HVP)   9 |----------| 6 (RESET#/PB3)         
          |          |                        
                                              
-[R]-  =  a 220 - 1K Ohm resistor            
this picture : 2011/12/08 by pcm1723         
modified :2015/02/27 by KD                                                    

thanks to pcm1723 for tpitest.pde upon which  

this is based                                 
