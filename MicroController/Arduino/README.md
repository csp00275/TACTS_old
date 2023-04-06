Data processing with Arduino

If you want to use multiple multiplexers, be sure to turn off all multiplexers other than the one you want to use.

If you want to use 0x70, then you should turn off 0x71, 0x72 etc.... 

turn off means "Transmitting the 0x00" data on multiplexor ( 0x00 = 0b00000000 it means turning off all channel)
