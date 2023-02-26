# RP2040_Midi_Scanner
Midi Scanner based on the Raspberry Pi Pico
(C) 2019-2023 David J Bottrill, Shady Grove Electronics

  IPMidi or USBMidi Master/Slave scanner for Raspberry Pi Pico board uses I2C BUS to connect multiple slave scanners
  boards can be programmed to be either a Master or slave, the master board can also be used as a keyboard or piston scanner. 
  
  This is designed to be compatible and complement digital pipe organs based on GrandOrgue or Hauptwerk.
  
  The I2C bus is configured to run at 4MHz to reduce latency and the polling interval can be changed in 
  the default settings section if required.
  
  The hardware is designed to connect the boards via a 10 way ribbon cable with odd cables being ground. The hardare has been tested
  with ribbon cables of uo to 4M length without problems, I2C pullup resistors of 10K ohms and fitted on each scanner board.
  
  I2C protocol polls slave devices for 2 bytes at a time if the buffer is empty the slave returns
  the Midi channel and information about the slave type.
  
  If there is Midi data then this is also encoded in 2 bytes.
  
  velocity is always assumed to be 127 although this default can be changed.
  
  The first byte is organised as follows:
  
  00        NooP returns Midi channel in the first byte and the scanner capabilites in the second byte
  
  10 - 3F - Midi controller NPRN command the first 4 are reserved for swell channels and the rest Illuminated Pistons
  
  40 -      Midi note off command, note is in the second byte
  
  41 -      Midi note on commnand, note is in the second byte
  
  42 -      Midi programe change command, program number is in the second byte, this is no longer used in this version
  If the MSB is set this indicates that there are further Midi commands in the send buffer
  
  Sending 0 , 0 to a remote device will reset it's buffer
  
  Sending 1 , 0 to a remote device will enable scanning and force a re-scan of any conneted analogue controllers.
  
  The above is only used to initially assess the remote devices' capabilities and then start them scanning.
  
  If the I2C bus address on the board is configured as 0 the board will configure itself as a master scanner
  there can be only one Master scanner on the bus, all other boards must be configured as slaves.
  
  The master scanner will poll each remote device in ascending order of I2C address
  the buffer will be emptied on the slave scanner before moving onto the next slave scanner.
  
  Asynchronously any received Midi commands are transmitted over IPMidi in a single 
  multicast packet of variable size up to the maximum set by UDP_TX_PACKET_MAX_SIZE and the midi buffer has been emptied.
  The UDP destination port is 21928 which equates to the first IP Midi port, this can be changed
  by the setting destport = 21929 etc.
  
  The scanner uses a Wiznet W5500 compatible Ethernet adapter. 
  
  The master scanner will listen to IPMidi broadcasts and will update any Illuminated pistons by sending I2C commands   
  to the relevant slave.
  
  If no Ethernet hardware is found or there is no ethernet cable is connected or the adapater fails to obtain and IP address via DHCP
  then the board will fallback to USB Midi and will pause with the on-board LED flashing with a short on and a long off flash until
  the Midi port is connected.
  
  At boot time the board waits for 5 seconds during which time the on-board LED will flash, if a USB Serial connection
  is made during this time then the board will go into diagnostic (debug) mode allowing for changes to settings to be made 
  and saved to EEPROM and in addition during operation debug messages will be send to the serial console.
  
  Once Setup has competed the on-board LED will go out and will flash briefly for each midi event processed. 
   
  The scanner can be configured for 8x8 diode matrix keyboards such as those made by Fatar and the pinout is compatible with
  commercial "FatBreak" connectors, or it can drive an 8x8 array of inexpensive ITR9606-F Opto-Switches. 
  
  Alternatively the board can connect to 16 LED illuminated Piston buttons and uses an innovative approach to use 
  a single GPIO pin to both operate the LED and read the push button switch.
       
  The board supports up to 3 expression pedals and can be configured for Linear or Logarithmic swell potentiometers 
  or for my own custom designed and 3D printed expression pedals that use a time of flight distance sensor and require 
  a tangent calculation to convert pedal height to pedal angle.
  
  The board should be compiled using the excellent arduino-pico core: https://github.com/earlephilhower/arduino-pico 
  remember to configure the compiler to use the Adafruit Tiny USB stack instead of the default Pico SDK USB Stack else the compiler 
  will fail.
  
  Both microcontroller cores of the RP2040 are used, the code is split between the microcontroller cores in an attempt to maximise
  performance.
  
  V7.1.0
  * Many bugfixes and improvements
  * Waits for 5 seconds for USB serial to connect at boot time in order to enter debug / congigure mode
  * If an NRPN control change message is received by the master this will trigger a re-scan of any analogue controllers on all boards
  * USB Midi now handles NRPN messages  
  * The hardware I2C bus reset line (GPIO 22) has been removed the master now just sends I2C scan stop and restart commands to the slaves
  * If no Ethernet hardware is detected or DHCP fails will default to USB Midi
  * ethernet.maintain() is executed every 100mS as per guidelines this ensures DHCP renews seamlessly
  * The code is distributed better over both cores of the RP2040 with most of the housekkeeping and I/O running on core 0
    and the keyboard, piston and analogue controller scanning running on core 1
  
