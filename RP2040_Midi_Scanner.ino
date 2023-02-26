/*
  (C) 2019-2023 David J Bottrill, Shady Grove Electronics

  IPMidi or USBMidi Master/Slave scanner for Raspberry Pi Pico board uses I2C BUS to connect multiple slave scanners
  boards can be programmed to be either a Master or slave, the master board can also be used as a keyboard or piston scanner. 
  This is designed to be compatible and complement digital pipe organes based on GrandOrgue or Hauptwerk.

  The I2C bus is configured to run at 4MHz to reduce latency and the polling interval can be changed in 
  the default settings section if required.
  The hardware is designed to connect the boards via a 10 way ribbon cable with odd cables being ground. The hardare has been tested
  with ribbon cables of uo to 4M length without problems, I2C pullup resistors of 10K ohms and fitted on each scanner board.

  I2C protocol polls slave devices for 2 bytes at a time if the buffer is empty the slave returns
  the Midi channel and information about the slave type.
  If there is Midi data then this is also encoded in 2 bytes.
  velocity is always assumed to be 127 although this default can be changed.
  The first byte is organised as follows:
  00        NooP returns Midi channel in the fist byte and the scanner capabilites in the second byte
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
  * Waits for 5 seconds for USB serial to connect at boot time  
  * If an NRPN control change message is received by the master this will trigger a re-scan of any analogue controllers on all boards
  * USB Midi now handles NRPN messages  
  * The hardware I2C bus reset line (GPIO 22) has been removed the master now just sends I2C scan stop and restart commands to the slaves
  * If no Ethernet hardware is detected or DHCP fails will default to USB Midi
  * ethernet.maintain() is executed every 100mS as per guidelines this ensures DHCP renews seamlessly
  * The code is distributed better over both cores of the RP2040 with most of the housekkeeping and I/O running on core 0
    and the keyboard, piston and analogue controller scanning running on core 1

*/

#include "rp2040.h"  //Midi board V7a definitions for Raspberry Pi Pico Microcontroller

#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
EthernetUDP Udp;  //An EthernetUDP instance to let us send and receive packets over IPMidi

#include <MIDI.h>
#include <Adafruit_TinyUSB.h>
Adafruit_USBD_MIDI usb_midi;  //USB MIDI object
// Create a new instance of the Arduino MIDI Library and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

//*********************************************************************************************
//****                        Device defaults and setttings                                ****
//*********************************************************************************************
#define codeRev "V7.1.0"
byte kbtype = 0;                     //Default Keyboard type, 0 = Matrix 1 = serial 2 = Optical 3 = Illuminated Pistons
byte i2caddr = 0;                    //Default I2C device address
byte chan = 1;                       //Default Midi Channel number
byte swells = 0;                     //Default Number of expression inputs
uint16_t nrpnOffset = 1000;          //NRPN Starting parameter number default = 1000
#define velocity 127                 //Default Midi Velocity
#define LedDelay 50                  //LED_BUILTIN Flash timer (mS)
#define optoDelay 350                //Optical Keyboard scan row delay (uS)
#define mechDelay 100                //Mechanical Keyboard scan row delay (uS)
#define pistonDelay 5000             //Illuminated Piston scan delay (uS)
#define swellDelay 2000              //Analogue sample delay (uS)
#define swellmax 100                 //Number of scans to average swell value over
#define swellSens 3                  //Swell midi granularity
#define i2cDelay 1000                //I2c delay timer
#define ethDelay 2000                //Ethernet Poll delay (uS)
#define UDP_TX_PACKET_MAX_SIZE 768   //increase UDP packet size so could hold a full buffer !
#define dhcpDelay 100                //100 mS timer for ethernet.maintain() checking
#define swellDisplayDelay 1000       //Swell display on LED timer (mS)
IPAddress multicast(225, 0, 0, 37);  //IPMidi broadcast address
unsigned int destPort = 21928;       //IPMidi Port, 21928 for port 0 21929 for port 1 etc
#define debugDelay 20                //Number of counts to wait for the serial port to connect 20 = 5 Seconds


//*********************************************************************************************
//****                              Global Variables                                       ****
//*********************************************************************************************
byte mac[6];
bool noeth = false;
bool badeeprom = true;

byte swellscan = 0;                       //Swell scan counter
int rowidx = 0;                           //Keyboard row inde
unsigned long int swell[4];               //Swell value array
byte Oswell[4] = { 128, 128, 128, 128 };  //Old swell values array
int Aswell[4];                            //Swell value accumulator
byte rxswell[4];                          //Received swell value
uint16_t sCal[3][4] = {
  //Swell calibration table
  0, 0, 0, 0,  //Swell Type 0 = not used, 1 = Linear, 2 = Log, 3 = Tangent
  0, 0, 0, 0,  //Zero offset
  0, 0, 0, 0   //Max Value
};

int tTable[4][128];  //Calculated translate table
int sLow[4];
int sHigh[4];
int sAve[4];

byte keybuf[3][128] = {};  //Key buffers: 0 = rawbuf, 1 = keybuf, 2 = keystat

uint8_t nrpnL[16];  //NRPN Low byte
uint8_t nrpnH[16];  //NRPN High byte

uint32_t ledstat = 0xffffffff;  //LED Output buffer to off
volatile byte txbuf[3][256];    //Output Circular buffer
byte inidx = 0;                 //Buffer input pointer
byte outidx = 0;                //Buffer output pointer
volatile byte i2cbuf[2][256];   //I2C receive Circular buffer
byte i2cinidx = 0;              //I2C Buffer input pointer
byte i2coutidx = 0;             //I2C Buffer output pointer
byte rxa, rxb;
byte type;
bool debug = true;  //Debug status flag
unsigned int kbdDelay;
unsigned long kbdTime;
unsigned long pistonTime;
unsigned long swellTime;
unsigned long dhcpTime;
unsigned long i2cTime;
unsigned long ethTime;
unsigned long LED_BUILTINTime;

int cc;
int i2cfile[128];
byte i2cchan[128];
byte i2ctype[128];
byte i2cctr;
byte i2cmax;
byte error;
byte address;
bool scanning = false;

const float logTable[] = {  //LOG10(x) 130 values needed for Hammond optical swell pedals calculations

  0.000000, 0.301030, 0.477121, 0.602060, 0.698970, 0.778151, 0.845098, 0.903090,
  0.954243, 1.000000, 1.041393, 1.079181, 1.113943, 1.146128, 1.176091, 1.204120,
  1.230449, 1.255273, 1.278754, 1.301030, 1.322219, 1.342423, 1.361728, 1.380211,
  1.397940, 1.414973, 1.431364, 1.447158, 1.462398, 1.477121, 1.491362, 1.505150,
  1.518514, 1.531479, 1.544068, 1.556303, 1.568202, 1.579784, 1.591065, 1.602060,
  1.612784, 1.623249, 1.633468, 1.643453, 1.653213, 1.662758, 1.672098, 1.681241,
  1.690196, 1.698970, 1.707570, 1.716003, 1.724276, 1.732394, 1.740363, 1.748188,
  1.755875, 1.763428, 1.770852, 1.778151, 1.785330, 1.792392, 1.799341, 1.806180,
  1.812913, 1.819544, 1.826075, 1.832509, 1.838849, 1.845098, 1.851258, 1.857332,
  1.863323, 1.869232, 1.875061, 1.880814, 1.886491, 1.892095, 1.897627, 1.903090,
  1.908485, 1.913814, 1.919078, 1.924279, 1.929419, 1.934498, 1.939519, 1.944483,
  1.949390, 1.954243, 1.959041, 1.963788, 1.968483, 1.973128, 1.977724, 1.982271,
  1.986772, 1.991226, 1.995635, 2.000000, 2.004321, 2.008600, 2.012837, 2.017033,
  2.021189, 2.025306, 2.029384, 2.033424, 2.037426, 2.041393, 2.045323, 2.049218,
  2.053078, 2.056905, 2.060698, 2.064458, 2.068186, 2.071882, 2.075547, 2.079181,
  2.082785, 2.086360, 2.089905, 2.093422, 2.096910, 2.100371, 2.103804, 2.107210
};


const float tanTable[] = {  //Tangent table for converting pedal height to angle for Pololu (Sharp) time of flight sensors

  0.000000, 0.011707, 0.023450, 0.035228, 0.047043, 0.058895, 0.070785, 0.082713,
  0.094679, 0.106685, 0.118731, 0.130818, 0.142946, 0.155115, 0.167327, 0.179582,
  0.191881, 0.204224, 0.216612, 0.229046, 0.241526, 0.254053, 0.266628, 0.279251,
  0.291924, 0.304646, 0.317418, 0.330243, 0.343119, 0.356047, 0.369030, 0.382066,
  0.395158, 0.408306, 0.421510, 0.434772, 0.448092, 0.461471, 0.474910, 0.488410,
  0.501972, 0.515597, 0.529285, 0.543037, 0.556854, 0.570738, 0.584689, 0.598709,
  0.612797, 0.626955, 0.641185, 0.655487, 0.669861, 0.684311, 0.698835, 0.713436,
  0.728114, 0.742871, 0.757707, 0.772624, 0.787624, 0.802706, 0.817873, 0.833126,
  0.848465, 0.863892, 0.879409, 0.895017, 0.910717, 0.926510, 0.942397, 0.958381,
  0.974463, 0.990643, 1.006924, 1.023307, 1.039794, 1.056385, 1.073083, 1.089889,
  1.106805, 1.123832, 1.140973, 1.158228, 1.175600, 1.193090, 1.210700, 1.228433,
  1.246289, 1.264271, 1.282381, 1.300620, 1.318991, 1.337496, 1.356136, 1.374915,
  1.393834, 1.412895, 1.432101, 1.451453, 1.470955, 1.490609, 1.510416, 1.530381,
  1.550504, 1.570789, 1.591239, 1.611855, 1.632642, 1.653601, 1.674736, 1.696049,
  1.717543, 1.739222, 1.761089, 1.783146, 1.805397, 1.827846, 1.850496, 1.873349,
  1.896411, 1.919684, 1.943172, 1.966879, 1.990809, 2.014966, 2.039353, 2.063976
};

//Function Protoypes
void handleNoteOn(byte, byte, byte);
void handleNoteOff(byte, byte, byte);
void handleControlChange(byte, byte, byte);

//*********************************************************************************************
//****                                Setup function                                       ****
//*********************************************************************************************
void setup() {

  analogReadResolution(12);  //Set ADC resolution
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledOn);

#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  //Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  //Not needed if using the arduino-pico core
  TinyUSB_Device_Init(0);
#endif
  //Set Midi device name although PCs and Macs will remember the device when it first connects
  //so this may not appear to work on the software development PC
  usb_midi.setStringDescriptor("ShadyGrove");

  // Initialize MIDI, and listen to all MIDI channels
  // This will also call usb_midi's begin()
  MIDI.begin(MIDI_CHANNEL_OMNI);

  // Attach the handleNoteOn function to the MIDI Library. It will
  // be called whenever the board receives a MIDI Note On messages.
  MIDI.setHandleNoteOn(handleNoteOn);

  // Do the same for MIDI Note Off messages.
  MIDI.setHandleNoteOff(handleNoteOff);

  // And similarly for Continous Control messages.
  MIDI.setHandleControlChange(handleCC);

  // wait until device mounted this is done later only if Ethernet is not found or plugged in
  //while (!TinyUSBDevice.mounted()) delay(1);


  //*********************************************************************************************
  //****                          Set Random MAC address                                     ****
  //*********************************************************************************************
  randomSeed(analogReadTemp());  //Generate random MAC address
  mac[0] = 42;
  mac[1] = 43;
  mac[2] = random(256);
  mac[3] = random(256);
  mac[4] = random(256);
  mac[5] = random(256);


  //*********************************************************************************************
  //****                         Restore settting from EEPROM                                ****
  //*********************************************************************************************
  EEPROM.begin(512);
  // Various tests to determine if the EEPROM contents appear valid
  if (EEPROM.read(0) == 42 && EEPROM.read(1) == 43 && EEPROM.read(6) < 0x80 && EEPROM.read(7) < 16) {
    loadSettings();  //Restore Settings from EEPROM
    badeeprom = false;
  } else {
    saveSettings();  //EEPROM data invalid so save default settings
    badeeprom = true;
  }


  //*********************************************************************************************
  //****                         Start Serial Port if needed                                 ****
  //*********************************************************************************************
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  int ii = debugDelay;
  while (ii > 0) {                     //Wait for serial port to connect
    digitalWrite(LED_BUILTIN, ledOn);  //Flash the LED until Serial connects
    delay(125);
    digitalWrite(LED_BUILTIN, ledOff);  //Flash the LED until Serial connects;
    delay(125);
    if (Serial) ii = 1;  //Bail out if serial port connects
    ii--;
  }
  debug = Serial;                    //Set debug status accoring to whether the Serial port has connected
  digitalWrite(LED_BUILTIN, ledOn);  //Status LED on until setup complete
  delay(2000);

  //*********************************************************************************************
  //****                      Print Diagnostics if in Debug Mode                             ****
  //*********************************************************************************************
  if (debug == true) {
    Serial.write(27);   //Print "esc"
    Serial.print("c");  //Send esc c to reset screen
    Serial.print("\nShady Grove I2C Midi Master / Slave Keyboard Scanner ");
    Serial.print(codeRev);
    Serial.print(" ");
    Serial.print(__DATE__);
    Serial.print(" ");
    Serial.println(__TIME__);
    Serial.println();
  }



  //*********************************************************************************************
  //****                      In Debug Mode present config menu                              ****
  //*********************************************************************************************
  if (debug == true) {
    if (badeeprom == true) {
      Serial.println("EEPROM read failed, setting defaults");
      badeeprom = false;
    }
    while (cc != 6) {
      Serial.println("\nSelect Option:");
      Serial.printf("1. I2C Address: 0x%x\n\r", i2caddr);
      Serial.printf("2. Midi Channel: %d\n\r", chan);
      Serial.printf("3. Calibrate Controllers \n\r");
      Serial.print("4. Type: ");
      switch (kbtype) {
        case 0:
          Serial.println("Matrix");
          break;
        case 1:
          Serial.println("Serial");
          break;
        case 2:
          Serial.println("Optical");
          break;
        case 3:
          Serial.println("Illuminated Pistons");
          break;
        case 4:
          Serial.println("Master only");
          break;
        default:
          Serial.println("Not Set");
          break;
      }
      Serial.println("5. Save Settings and Exit");
      Serial.println("6. Exit");

      cc = getInput().toInt();

      switch (cc) {
        case (1):
          Serial.println("\n\rEnter I2C address in HEX (0 - 7F), 0 if Master Scanner");
          cc = hexToDec(getInput());
          if (cc > 127) cc = 127;
          if (cc < 0) cc = 0;
          i2caddr = cc;
          cc = 0;
          break;
        case (2):
          Serial.println("\n\rEnter Midi Channel");
          cc = getInput().toInt();
          if (cc > 15) cc = 15;
          if (cc < 0) cc = 0;
          chan = cc;
          cc = 0;
          break;
        case (3):
          calSwells();
          swells = 0;
          for (int x = 0; x < maxSwells; x++) {
            if (sCal[0][x] > 0) swells++;  //Set swells according to the data in sCal[0]
          }
          break;
        case (4):
          Serial.println("\n\rEnter 0 for Matrix, 2 for Optical, 3 for Illuminated Pistons or 4 for none");
          cc = getInput().toInt();
          if (cc > 4) cc = 4;
          if (cc < 0) cc = 0;
          kbtype = cc;
          cc = 0;
          if (kbtype == 1) kbtype = 2;  //We don't support serial keyboards in this version
          break;
        case (5):
          Serial.println();
          saveSettings();
          cc = 6;
          break;
        default:
          cc = 6;
          break;
      }
    }
    Serial.println();
  }
  calcTables();  //Calculate Swell Translate tables


  //*********************************************************************************************
  //****                                 Start I2C                                           ****
  //*********************************************************************************************

  Wire.setSDA(I2CSDA);
  Wire.setSCL(I2CSCL);

  if (i2caddr == 0) {
    Wire.begin();  //Join I2C bus as master
    Wire.setClock(400000);
  } else {
    Wire.begin(i2caddr);           //join I2C bus as slave
    Wire.onRequest(requestEvent);  //register event
    Wire.onReceive(receiveEvent);  //register event
  }

  //*********************************************************************************************
  // ****    If I2c address = 0 then configure as master and start Ethernet or USBMidi.      ****
  //*********************************************************************************************
  if (i2caddr == 0) {  //If Master scanner then go ahead and setup Ethernet etc.


    //*********************************************************************************************
    //****                          Start Ethernet or USB Midi                                 ****
    //*********************************************************************************************

    Ethernet.init(SS_W5500);
    // start the Ethernet connection:
    if (debug == true) Serial.println("Initialise Ethernet with DHCP");
    noeth = false;
    if (Ethernet.begin(mac) == 0) {
      noeth = true;

      if (debug == true) {
        Serial.println("Failed to configure Ethernet using DHCP");
        if (Ethernet.hardwareStatus() == EthernetNoHardware) Serial.println("Ethernet hardware was not found");
        if (Ethernet.linkStatus() == LinkOFF) Serial.println("Ethernet cable is not connected");
        Serial.println("No Ethernet so falling back to USB Midi");
      }

      while (!TinyUSBDevice.mounted()) {    //Make sure USB Midi has connected before continuing
        digitalWrite(LED_BUILTIN, ledOff);  //Flash the LED Short on Long oof
        delay(400);                         //until USB Midi connects
        digitalWrite(LED_BUILTIN, ledOn);
        delay(100);
      }

      digitalWrite(LED_BUILTIN, ledOn);
      if (debug) Serial.println("USB Midi Connected");
    }


    //*********************************************************************************************
    //****                              Start UDP Multicast                                    ****
    //*********************************************************************************************
    if (noeth == false) {
      if (debug == true) Serial.println("Starting IPMidi");
      Udp.beginMulticast(multicast, destPort);
    }

    //*********************************************************************************************
    //****                   If in debug mode print Ethernet information                       ****
    //*********************************************************************************************
    if (debug == true && noeth == false) {
      Serial.printf("MAC address: %x:%x:%x:%x:%x:%x\n\r", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      Serial.print("IP address: ");
      Serial.println(Ethernet.localIP());
      Serial.print("Multicast address: ");
      Serial.print(multicast);
      Serial.printf("  Port: %d\n\r\n", destPort);
      Serial.println();
    }
  }


  if (i2caddr == 0) {

    //*********************************************************************************************
    //****                      Scan I2C Bus for slave modules                                 ****
    //*********************************************************************************************
    if (debug == true) Serial.println("Looking for Scanner Devices");
    i2cmax = 0;
    for (address = 1; address < 127; address++) {
      // The i2c_scanner uses the return value of Write.endTransmisstion to see if
      // a device did acknowledge on the address.

      Wire.beginTransmission(address);
      error = Wire.endTransmission();

      if (error == 0) {
        i2cfile[i2cmax] = address;
        i2cmax++;
      }
    }
    if (debug == true) Serial.printf("\nFound %d additional slave I2C Midi Scanner modules\n\n\r", i2cmax);


    //*********************************************************************************************
    //****                       Interrogate and start slave modules                           ****
    //*********************************************************************************************
    //Print master information

    if (debug == true) {
      Serial.printf("   Master scanner     Channel: %d", chan);
      switch (kbtype) {
        case 0:
          Serial.print("  Matrix  Scanner");
          break;
        case 1:
          Serial.print("  Serial  Scanner");
          break;
        case 2:
          Serial.print("  Optical Scanner");
          break;
        case 3:
          Serial.print("  Piston  Scanner");
          break;
        case 4:
          Serial.print("  No Keyboard    ");
          break;
      }
      Serial.printf(" with %d Controllers\n\r", swells);
    }

    //Print slave information
    for (i2cctr = 0; i2cctr < i2cmax; i2cctr++) {
      //Send clear command
      Wire.beginTransmission(i2cfile[i2cctr]);
      Wire.write(0);
      Wire.write(0);
      Wire.endTransmission();  // stop transmitting
      delayMicroseconds(500);
      //Get i2c module configuration
      Wire.requestFrom(i2cfile[i2cctr], 2);  // request 2 bytes from slave device
      i2cchan[i2cctr] = Wire.read();
      i2ctype[i2cctr] = Wire.read();

      if (debug == true) {
        Serial.printf("%d: I2C Address: 0x%x  Channel: %d", i2cctr + 1, i2cfile[i2cctr], i2cchan[i2cctr]);
        switch ((i2ctype[i2cctr] & 0x07)) {
          case 0:
            Serial.print("  Matrix  Scanner");
            break;
          case 1:
            Serial.print("  Serial  Scanner");
            break;
          case 2:
            Serial.print("  Optical Scanner");
            break;
          case 3:
            Serial.print("  Illuminated Pistons");
            break;
        }
        Serial.printf(" with %d Controllers\n\r", (i2ctype[i2cctr] >> 3));
      }

      //Send enable scan command
      Wire.beginTransmission(i2cfile[i2cctr]);
      Wire.write(1);
      Wire.write(0);
      Wire.endTransmission();  // stop transmitting
      delayMicroseconds(500);
    }
  }

  //*********************************************************************************************
  //****                          Get ready to start scanning                                ****
  //*********************************************************************************************


  if (kbtype < 3) {  //No need to do anything here for illuminated pistons

    //setup to be compatible with a FATBREAK connected keyboard or pistons 64  keys / pistons

    int ii;
    for (ii = 0; ii < 8; ii++) {
      pinMode(cols[ii], INPUT_PULLUP);
      pinMode(rows[ii], OUTPUT_12MA);
      digitalWrite(rows[ii], HIGH);
    }
  }

  //Build scanner type status byte to show capabilities
  type = kbtype & 0x07;  //Lowest 3 bits = keyboard type
  type += swells * 8;    //Next 3 bits = number of analogue controllers
  if (debug == true) Serial.println("\nResetting Buffer");
  inidx = outidx;        //Reset Buffer
  i2cinidx = i2coutidx;  //Reset Buffer

  digitalWrite(LED_BUILTIN, ledOff);  //Status LED off as we're ready to start scanning
  if (debug == true) Serial.println("\nScanning...\n");

  if (kbtype == 0) {  //Set keyboard display based on type
    kbdDelay = mechDelay;
  } else {
    kbdDelay = optoDelay;
  }

  scanning = true;  //Set the flag to start scanning

}  //end of Setup (core 0)



//*********************************************************************************************
//****     Main Loop Core 0 handles Ethernet, USB, I2C and analogue controller tasks       ****
//*********************************************************************************************
void loop() {
  if (i2caddr == 0) {
    //*********************************************************************************************
    //****                      Is it time check for DHCP renewal                              ****
    //*********************************************************************************************
    if (noeth == false) {
      if (millis() < dhcpTime) dhcpTime = 0;
      if (millis() > dhcpTime + dhcpDelay) {  //check for DHCP renewal every 100mS (default)
        dhcpTime = millis();
        uint16_t e = Ethernet.maintain();
        if (debug) {  //In debug mode print DHCP renew information
          switch (e) {
            case 1:
              //renewed fail
              Serial.println("DHCP Error: renew fail");
              break;

            case 2:
              //renewed success
              Serial.print("DHCP Renew success: ");
              //print your local IP address:
              Serial.println(Ethernet.localIP());
              break;

            case 3:
              //rebind fail
              Serial.println("DHCP Error: rebind fail");
              break;

            case 4:
              //rebind success
              Serial.print("DHCP Rebind success: ");
              //print your local IP address:
              Serial.println(Ethernet.localIP());
              break;

            default:
              //nothing happened
              break;
          }
        }
      }
    }


    //*********************************************************************************************
    //****                  If it's time send and receive Midi messages                        ****
    //*********************************************************************************************
    if (micros() < ethTime) ethTime = 0;
    if (micros() > ethTime + ethDelay) {     //Is it time to send and receive over IPMidi ?
      ethTime = micros();                    //Store time check started
      if (i2caddr == 0 && noeth == false) {  //If Ethernet coonected then process
        IPmidiSend();                        //Send anything in transmit buffer over IPMidi
        IPmidiReceive();                     //Check for received Midi events and process
      } else {                               //No Ethernet so use USB
        USBmidiSend();                       //Send anything in transmit buffer over USBMidi
        MIDI.read();                         //read any new USB MIDI messages
      }                                      //these are handled by callbacks
    }

    //*********************************************************************************************
    //****                         Scan I2C Slaves if it's time                                ****
    //*********************************************************************************************
    if (i2caddr == 0) {  //If a master scanner the scan slaves
      if (micros() < i2cTime) i2cTime = 0;
      if (micros() > i2cTime + i2cDelay) {  //Wait for i2c loop delay
        i2cTime = micros();
        pollScanners();  //Poll slaves for new data
      }
    }
  }

  //*********************************************************************************************
  //****                        Scan Controllers if it's time                                ****
  //*********************************************************************************************
  if (swells > 0) {
    if (micros() < swellTime) swellTime = 0;
    if (micros() > swellTime + swellDelay) {  //Is it time to scan the Analogue inputs?
      swellTime = micros();                   //Store time Analogue scan started
      scanSwell();                            //Scan analogue inputs
    }
  }

  //*********************************************************************************************
  //****                 Turn off Status LED if the timeout has expired                      ****
  //*********************************************************************************************

  if (millis() > LED_BUILTINTime + LedDelay) {
    digitalWrite(LED_BUILTIN, ledOff);  //Turn LED off after a short delay
  }

}  //End of loop 0

//*********************************************************************************************
//****                 Core 1 handles all keyboard and piston scanning                     ****
//*********************************************************************************************
void setup1() {        //Core 1 setup, nothing to do here
  while (!scanning) {  //Wait for scanning to start
    delay(10);
  }
}

//*********************************************************************************************
//****                                Main Loop Core 1                                     ****
//*********************************************************************************************
void loop1() {

  //*********************************************************************************************
  //****                             Scan Keyboard if it's time                              ****
  //*********************************************************************************************

  if (kbtype == 0 || kbtype == 2) {
    if (micros() < kbdTime) kbdTime = 0;
    if (micros() > kbdTime + kbdDelay) {  //Is it time to scan the keyboard?
      kbdTime = micros();                 //Store time keyboard scan started
      rowidx = scanRow(keybuf, rowidx);   //Scan keyboard one row at a time (rowidx 0 to 7)
    }
  }


  //*********************************************************************************************
  //****                     Scan illuminated Pistons if it's time                           ****
  //*********************************************************************************************
  if (kbtype == 3) {
    if (micros() < pistonTime) pistonTime = 0;
    if (micros() > pistonTime + pistonDelay) {  //Is it time to scan the pistons?
      pistonTime = micros();                    //Store time piston scan started
      scanPistons();                            //Scan pistons
    }
  }


}  //End of Loop 1


//*********************************************************************************************
//****             Read Matrix or Optical keyboard a singele row at a time                 ****
//****                          Optical keyboards are inverted                             ****
//*********************************************************************************************

int scanRow(byte kbl[3][128], int r) {

  //read col GPIO pins into rawbuf
  int ki = (r * 8);
  if (kbtype == 2) {
    kbl[0][ki] = digitalRead(pxlte[0]);  //No need to invert Optical switches
    kbl[0][ki + 1] = digitalRead(cols[1]);
    kbl[0][ki + 2] = digitalRead(cols[2]);
    kbl[0][ki + 3] = digitalRead(cols[3]);
    kbl[0][ki + 4] = digitalRead(cols[4]);
    kbl[0][ki + 5] = digitalRead(cols[5]);
    kbl[0][ki + 6] = digitalRead(cols[6]);
    kbl[0][ki + 7] = digitalRead(cols[7]);
  } else {  //Invert mechanical switches
    kbl[0][ki]     = !digitalRead(cols[0]);
    kbl[0][ki + 1] = !digitalRead(cols[1]);
    kbl[0][ki + 2] = !digitalRead(cols[2]);
    kbl[0][ki + 3] = !digitalRead(cols[3]);
    kbl[0][ki + 4] = !digitalRead(cols[4]);
    kbl[0][ki + 5] = !digitalRead(cols[5]);
    kbl[0][ki + 6] = !digitalRead(cols[6]);
    kbl[0][ki + 7] = !digitalRead(cols[7]);
  }

  if (kbtype == 2 && r == 7) {  //phantom notes above top C off
    kbl[0][61] = 0;             //If we are scanning an optical
    kbl[0][62] = 0;             //Keyboard they need to be turned off
    kbl[0][63] = 0;
  }

  r++;  //Select row for next scan
  if (r > 7) r = 0;

  //Turn Row off (high) and next row on (low)
  if (r == 0) {
    digitalWrite(rows[7], HIGH);
  } else {
    digitalWrite(rows[r - 1], HIGH);
  }
  digitalWrite(rows[r], LOW);

  kProcess(kbl, ki, ki + 8, kbtype, chan);  //Process de-bounce and notes on and off
  return r;
}

//*********************************************************************************************
//****                      Read and process Illuminated Pistons                           ****
//*********************************************************************************************
void scanPistons(void) {
  for (int k = 0; k < 16; k++) {                  //Loop through 16 pistons
    pinMode(pxlte[k], INPUT_PULLUP);              //Set GPIO to Input
    keybuf[0][k + 64] = !digitalRead(pxlte[k]);   //Read the GPIO
    pinMode(pxlte[k], OUTPUT_4MA);                //Set GPIO back to output (4mA is ok for most illuminated buttons)
    digitalWrite(pxlte[k], bitRead(ledstat, k));  //Update LED to latest state
  }
  kProcess(keybuf, 64, 80, 0, chan);  //De-Bounce and process new button status
}

//*********************************************************************************************
//****                        Process de-bounce and notes on and off                       ****
//*********************************************************************************************
void kProcess(byte kbl[3][128], int k1, int k2, int k3, int c) {
  //Keyboard buffer pointer, Start Key, End Key, Keyboard type, Midi Channel
  byte note;       //Midi Note value
  int keyidx = 0;  //Key array index

  //De-Bounce mechanical key switches
  for (keyidx = k1; keyidx < k2; keyidx++) {
    if (kbl[0][keyidx] == 0) {  //Key pressed ?
      //Key not pressed
      if (kbl[1][keyidx] > 0) {                          //Is de-bounce count > 0
        kbl[1][keyidx]--;                                //Decrement de-bounce count
        if (k3 == 2 && keyidx < 61) kbl[1][keyidx] = 1;  //Optical keyboard = no de-bounce
      }
    } else {
      //key pressed
      if (kbl[1][keyidx] < 2) {
        kbl[1][keyidx]++;
        if (k3 == 2 && keyidx < 61) kbl[1][keyidx] = 2;  //Optical keyboard = no de-bounce
      }
    }

    //Process notes off and on
    if (kbl[2][keyidx] == 1) {  //Is note aleady on?
      //Yes
      if (kbl[1][keyidx] == 1) {                            //Is key off debounce done ?
        if (keyidx < 61) masterSend(0x40, keyidx + 36, c);  //Send note off
        kbl[1][keyidx] = 0;                                 //Set key value to 0 to indicate midi off sent
        kbl[2][keyidx] = 0;                                 //Set status of note to off
      }
    } else {                                                //Note is off
      if (kbl[1][keyidx] == 2) {                            //Has de-bounce completed
        if (keyidx < 61) masterSend(0x41, keyidx + 36, c);  //Send note on
        if (keyidx > 63) masterSend(keyidx - 60, 127, c);   //Send contoller change for Piston 4 - 36
        kbl[1][keyidx] = 3;                                 //Set de-bounce value to 3
        kbl[2][keyidx] = 1;                                 //Set status of note to on
      }
    }
  }
}


//*********************************************************************************************
//****         function that executes whenever data is received from I2C master            ****
//****               this function is registered as an event, see setup()                  ****
//*********************************************************************************************
void receiveEvent(int howMany) {
  while (1 < Wire.available()) {  //loop through all but the last byte
    rxa = Wire.read();            //receive byte
  }
  rxb = Wire.read();  //receive last byte

  if (rxa == 0 && rxb == 0) {
    if (debug == true) Serial.println("Scanning halted by Master");
    scanning = false;
    i2coutidx = 0;  //Reset I2C buffer
    i2cinidx = 0;
  }
  if (rxa == 1 && rxb == 0) {  //Restart requested by master
    if (debug == true) Serial.println("Scanning restarted by Master");
    scanning = true;
    Oswell[0] = 128;  //force re-scan of all swell contollers
    Oswell[1] = 128;
    Oswell[2] = 128;
    Oswell[3] = 128;  //Not needed on the pico but retained for future use
  }

  //Use received contoller values to turn on and off Piston LEDs
  //Contollers 0 - 3 and reserved for swell pedals 4 - 35 are reserved for Piston LEDs
  //This equates to rxa being 16-19 for swell pedals and 20-51 for Piston controllers
  if (debug == true) Serial.printf("RX Controller\t%d\t%d\t%d\n\r", chan, rxa - 16, rxb);
  if (rxa > 15 && rxa < 20) rxswell[rxa - 16] = rxb;  //Store received Swell contoller value
  if (rxa > 19 && rxa < 52) {
    if (rxb == 0) {
      bitWrite(ledstat, rxa - 20, 1);
    } else {
      bitWrite(ledstat, rxa - 20, 0);
    }
  } 
}


//*********************************************************************************************
//****       function that executes whenever data is requested by I2C master               ****
//****            this function is registered as an event, see setup()                     ****
//*********************************************************************************************
void requestEvent() {
  byte buf[2];
  if (rxa + rxb == 0) i2coutidx = i2cinidx;  //if in reset state reset buffer
  if (i2cinidx == i2coutidx) {               //If buffer empty return Midi channel plus scanner capabilities                                       //Bits 3 and 4 show swell count
    buf[0] = chan;
    buf[1] = type;  //return scanner capabilities
  } else {
    buf[0] = i2cbuf[0][i2coutidx];
    buf[1] = i2cbuf[1][i2coutidx];
    i2coutidx++;
    if (i2coutidx != i2cinidx) buf[0] = buf[0] | 0x80;  //If the buffer isn't empty set MSB
  }
  Wire.write(buf, 2);
}

//*********************************************************************************************
// ****                Poll each I2C module in turn for Midi events                        ****
// ****                          and place in circular buffer                              ****
//*********************************************************************************************
void pollScanners() {
  byte rxa, rxb, ch;
  for (i2cctr = 0; i2cctr < i2cmax; i2cctr++) {
    bool more = 1;
    while (more == 1) {
      Wire.requestFrom(i2cfile[i2cctr], 2);  //request 2 bytes from slave device
      if (Wire.available()) rxa = Wire.read();
      if (Wire.available()) rxb = Wire.read();
      more = 0;
      if (rxa >= 0x80) more = 1;         //if MSB of a is 1 then the buffer isn't empty
      rxa = rxa & 0x7f;                  //mask off the buffer bit
      ch = i2cchan[i2cctr];              //Recover the channel number
      if (rxa > 15) {                    //Is message valid
        if (rxa < 0x40) rxa = rxa - 16;  //Swell controllers start at 0
        masterSend(rxa, rxb, ch);
      }
    }
  }  //Next I2C scanner
}


//*********************************************************************************************
// ****                   Function to restart all slave scanners                           ****
// ****     this will force all swell controllers to update and re-send NRPN values        ****
//*********************************************************************************************
void restartScanners() {
  for (i2cctr = 0; i2cctr < i2cmax; i2cctr++) {
    //Send enable scan command
    Wire.beginTransmission(i2cfile[i2cctr]);
    Wire.write(1);
    Wire.write(0);
    Wire.endTransmission();  //stop transmitting
  }                          //Next I2C scanner
}


//*********************************************************************************************
//****                     Function to add data to send buffer                             ****
//*********************************************************************************************
void masterSend(byte rxa, byte rxb, byte ch) {
  byte a;
  byte b;
  byte c;
  digitalWrite(LED_BUILTIN, ledOn);  //Turn on LED_BUILTIN
  LED_BUILTINTime = millis();
  switch (rxa) {
    case 0x40:  //Note off
      a = 0x80 | ch;
      b = rxb;
      c = velocity;
      if (debug == true) Serial.printf("TX Note Off\t%d\t%d\t%d\n\r", ch, b, c);
      break;
    case 0x41:  //Note on
      a = 0x90 | ch;
      b = rxb;
      c = velocity;
      if (debug == true) Serial.printf("TX Note On\t%d\t%d\t%d\n\r", ch, b, c);
      break;
    case 0x42:  //Program change not currently used
      a = 0xC0 | ch;
      b = rxb;
      c = 0xff;
      if (debug == true) Serial.printf("TX Program\t%d\t%d\n\r", ch, b);
      break;
    default:
      a = 0xB0 | ch;  //Controller value
      b = rxa;
      c = rxb;
      uint16_t n = b + nrpnOffset;  //Calculate the NRPN register number, the first 4 are reservered for swell pedals
      if (debug == true) Serial.printf("TX NRPN\t\t%d\t%d\t%d\n\r", ch, n, c);
      tx_bufferWrite(a, 99, n >> 7);          //Write NRPN High byte to output buffer
      tx_bufferWrite(a, 98, n & 0x7f);        //Write NRPN Low byte to output buffer
      if (b > 3) {                            //If a piston has been pressed force all swell controllers to update their values
        if (i2caddr == 0) restartScanners();  //if master force all slaves to re-scan swell controllers
        Oswell[0] = 128;                      //force master to re-scan swell contollers
        Oswell[1] = 128;
        Oswell[2] = 128;
        Oswell[3] = 128;  //Not needed on the Pico but retained for future use
      }
      b = 6;  //NRPN data register
  }
  if (rxa < 48) rxa = rxa + 16;  //if it's a contoller command add 16 to allow for a range of 16-63 (0-47)
  tx_bufferWrite(a, b, c);       //Write to midi buffer
  i2c_bufferWrite(rxa, rxb);     //Write to I2C buffer
}

//*********************************************************************************************
//****              Function to add data to circular Midi send buffer                      ****
//*********************************************************************************************
void tx_bufferWrite(uint8_t par1, uint8_t par2, uint8_t par3) {
  txbuf[0][inidx] = par1;  //Write to circular buffer
  txbuf[1][inidx] = par2;  //Write to circular buffer
  txbuf[2][inidx] = par3;  //Write to circular buffer
  inidx++;                 //Increment buffer input index
}


//*********************************************************************************************
//****               Function to add data to circular I2C send buffer                      ****
//*********************************************************************************************
void i2c_bufferWrite(uint8_t par1, uint8_t par2) {
  i2cbuf[0][i2cinidx] = par1;  //Write to circular buffer
  i2cbuf[1][i2cinidx] = par2;  //Write to circular buffer
  i2cinidx++;                  //Increment buffer input index
}



//*********************************************************************************************
//****                     Save Settings to simulated EEPROM                               ****
//*********************************************************************************************
void saveSettings(void) {
  if (debug == true) Serial.println("Saving Settings to EEPROM");

  EEPROM.write(0, 42);      //42 indicates settings are valid
  EEPROM.write(1, mac[1]);  //Save MAC address
  EEPROM.write(2, mac[2]);
  EEPROM.write(3, mac[3]);
  EEPROM.write(4, mac[4]);
  EEPROM.write(5, mac[5]);
  EEPROM.write(6, i2caddr);  //Save I2C address
  EEPROM.write(7, chan);     //Save Midi channel
  EEPROM.write(8, swells);   //Save number of swells
  EEPROM.write(9, kbtype);   //Save Keyboard type

  int ii = 10;
  for (int x = 0; x < 3; x++) {  //Save swell calibration data
    for (int y = 0; y < 4; y++) {
      EEPROM.write(ii, sCal[x][y]);
      ii++;
      EEPROM.write(ii, sCal[x][y] >> 8);
      ii++;
    }
  }
  EEPROM.commit();
}


//*********************************************************************************************
//****                 Restore Settings from simulated EEPROM                              ****
//*********************************************************************************************
void loadSettings(void) {
  if (debug == true) Serial.println("Restoring Settings from EEPROM");

  mac[0] = EEPROM.read(0);  //Restore MAC address;
  mac[1] = EEPROM.read(1);
  mac[2] = EEPROM.read(2);
  mac[3] = EEPROM.read(3);
  mac[4] = EEPROM.read(4);
  mac[5] = EEPROM.read(5);
  i2caddr = EEPROM.read(6);  //Restore I2C address;
  chan = EEPROM.read(7);     //Restore Midi channel
  swells = EEPROM.read(8);   //Restore number of swells
  kbtype = EEPROM.read(9);   //Restore Keyboard type

  int ii = 10;
  for (int x = 0; x < 3; x++) {  //Restore swell calibration data
    for (int y = 0; y < 4; y++) {
      sCal[x][y] = EEPROM.read(ii);
      ii++;
      sCal[x][y] = sCal[x][y] + EEPROM.read(ii) * 256;
      ii++;
    }
  }
}

//*********************************************************************************************
//****                 Calculate the swell pedal calibration tables                        ****
//*********************************************************************************************
void calcTables() {
  for (int y = 0; y < maxSwells; y++) {
    int tRange = sCal[2][y] - sCal[1][y];
    float tScale = tRange / 2;
    for (int x = 0; x < 128; x++) {
      if (sCal[0][y] == 3) tTable[y][x] = round(sCal[1][y] + (tanTable[x] * tScale));  //Logarithmic
      if (sCal[0][y] == 2) tTable[y][x] = round(sCal[1][y] + (logTable[x] * tScale));  //Logarithmic
      if (sCal[0][y] == 1) tTable[y][x] = sCal[1][y] + (x * round((tRange / 128)));    //Linear
      if (sCal[0][y] == 0) tTable[y][x] = 0;                                           //Not in use
    }
    if (debug && sCal[0][y] == 3) Serial.printf("Tangent lookup table for swell %d generated\n\r", y);
    if (debug && sCal[0][y] == 2) Serial.printf("Logarithmic lookup table for swell %d generated\n\r", y);
    if (debug && sCal[0][y] == 1) Serial.printf("Linear lookup table for swell %d generated\n\r", y);
  }
}



//*********************************************************************************************
//****                             Process Swell pedals                                    ****
//*********************************************************************************************
void scanSwell(void) {
  // Read Swell pedals
  if (sCal[0][0] > 0) Aswell[0] += analogRead(swell0);  //Get swell values
  if (sCal[0][1] > 0) Aswell[1] += analogRead(swell1);
  if (sCal[0][2] > 0) Aswell[2] += analogRead(swell2);
  if (sCal[0][3] > 0) Aswell[3] += analogRead(swell3);
  if (swellscan > swellmax) {              //Is it time to read swell pedals
    swellscan = 0;                         //Reset swell scan counter
    for (int x = 0; x < maxSwells; x++) {  //Test each swell pedal in turn
      if (sCal[0][x] > 0) {
        int v = round(Aswell[x] / swellmax);  //Calculate the average reading
        Aswell[x] = 0;

        for (int y = 0; y < 128; y++) {  //Scan through the lookup table
          swell[x] = y;
          if (v <= tTable[x][y]) {  //Find a match in the table
            y = 128;                //Bail out when we've got a match
          }
        }  //swell[] now has the value
        swell[x] /= swellSens;
        swell[x] *= swellSens;
        if (swell[x] == 126) swell[x] = 127;

        if (swell[x] != Oswell[x]) {      //Test if swell value has changed
          masterSend(x, swell[x], chan);  //Send Midi event to buffer 0 - 3
          Oswell[x] = swell[x];           //Save new swell value as old
        }
      }
    }  //Next swell pedal
  }
  swellscan++;  //Increment swell scan counter
}


//*********************************************************************************************
//****                  Calibrate the analogue expression inputs                           ****
//*********************************************************************************************
void calSwells() {
  int cc, x, y;
  swells = 0;
  Serial.println("Enter details of each swell controller in turn (4)");
  for (y = 0; y < maxSwells; y++) {
    Serial.printf("Swell %d enter type 0 = None, 1 = Linear, 2 = Logarithmic, 3 = Tangent\n\r", y + 1);
    cc = getInput().toInt();
    if (cc > 3) cc = 3;
    if (cc < 0) cc = 0;
    sCal[0][y] = cc;
    if (sCal[0][y] > 0) swells++;
  }

  Serial.println("Set all swells to minimum and press Return");
  getInput();
  getSwell();
  for (x = 0; x < maxSwells; x++) {
    if (sCal[0][x] > 0) sCal[1][x] = sAve[x] + (sHigh[x] - sAve[x]) / 2;
  }
  Serial.println("Set all swells to maximum and press Return");
  getInput();
  getSwell();
  for (x = 0; x < maxSwells; x++) {
    if (sCal[0][x] > 0) sCal[2][x] = sAve[x];
  }
  Serial.println("Calibration Done remember to Save Settings");
}


//*********************************************************************************************
//****                      Read swell pedals for Calibration                              ****
//*********************************************************************************************
void getSwell(void) {
  int x;
  int p[4];
  const int maxTest = 2000;
  for (x = 0; x < 4; x++) {
    Aswell[x] = 0;
    sLow[x] = 4095;
    sHigh[x] = 0;
  }
  for (int c = 0; c < maxTest; c++) {
    delayMicroseconds(swellDelay);
    p[0] = analogRead(swell0);
    p[1] = analogRead(swell1);
    p[2] = analogRead(swell2);
    p[3] = analogRead(swell3);
    for (x = 0; x < 4; x++) {
      Aswell[x] += p[x];
      if (p[x] < sLow[x]) sLow[x] = p[x];
      if (p[x] > sHigh[x]) sHigh[x] = p[x];
    }
  }

  if (debug) {
    for (x = 0; x < 4; x++) {
      sAve[x] = Aswell[x] / maxTest;  //Save average
      if (sCal[0][x] > 0) Serial.printf("Swell %d  Low: %d  High: %d  Ave: %d  %d Samples\n\r", x, sLow[x], sHigh[x], sAve[x], maxTest);
    }
    delay(2);
  }
}

//*********************************************************************************************
//****                       Serial input string function                                  ****
//*********************************************************************************************
String getInput() {
  bool gotS = false;
  String rs = "";
  char received;

  while (gotS == false) {
    while (Serial.available() > 0) {
      received = Serial.read();
      Serial.write(received);  //Echo input
      if (received == '\r' || received == '\n') {
        gotS = true;
      } else {
        rs += received;
      }
    }
  }
  return (rs);
}


//*********************************************************************************************
//****                           Convert HEX to Decimal                                    ****
//*********************************************************************************************
unsigned int hexToDec(String hexString) {
  unsigned int decValue = 0;
  int nextInt;
  for (int i = 0; i < hexString.length(); i++) {
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);

    decValue = (decValue * 16) + nextInt;
  }
  return decValue;
}


//*********************************************************************************************
// ****       If there are any Midi events in the circular buffer send it over IPMidi      ****
// ****                 Send multiple Midi messages in a single UDP packet                 ****
//*********************************************************************************************
void IPmidiSend(void) {
  char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold data packet
  if (inidx != outidx) {
    int bc = 0;
    while (inidx != outidx && bc < (UDP_TX_PACKET_MAX_SIZE - 3)) {
      packetBuffer[bc] = txbuf[0][outidx];
      bc++;
      packetBuffer[bc] = txbuf[1][outidx];
      bc++;
      if (txbuf[2][outidx] != 0xFF) {  //if it's a 3 byte message add the 3rd byte to buffer
        packetBuffer[bc] = txbuf[2][outidx];
        bc++;
      }
      outidx++;
    }
    Udp.beginPacket(multicast, destPort);
    Udp.write(packetBuffer, bc);
    Udp.endPacket();
  }
}

//*********************************************************************************************
// ****      If there are any Midi events in the circular buffer send it over USBMidi      ****
//*********************************************************************************************
void USBmidiSend(void) {

  if (inidx != outidx) {
    while (inidx != outidx) {
      if ((txbuf[0][outidx] & 0xf0) == 0x90) MIDI.sendNoteOn(txbuf[1][outidx], 127, (txbuf[0][outidx] & 0x0f) + 1);
      if ((txbuf[0][outidx] & 0xf0) == 0x80) MIDI.sendNoteOff(txbuf[1][outidx], 127, (txbuf[0][outidx] & 0x0f) + 1);
      if ((txbuf[0][outidx] & 0xf0) == 0xB0) MIDI.sendControlChange(txbuf[1][outidx], txbuf[2][outidx], (txbuf[0][outidx] & 0x0f) + 1);
      outidx++;
    }
  }
}

//*********************************************************************************************
//****                   Check if there's incomming IPMIDI data available                  ****
//****                 update the illuminated piston LEDs if required over I2C             ****
//*********************************************************************************************
void IPmidiReceive(void) {
  char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold data packet
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);

    for (int pii = 0; pii < packetSize; pii++) {

      if (packetBuffer[pii] > 127) {
        //Process Controller change message
        if ((packetBuffer[pii] & 0xF0) == 0xB0) {
          uint8_t ch = (packetBuffer[pii] & 0x0F);
          uint8_t par1 = packetBuffer[pii + 1];
          uint8_t par2 = packetBuffer[pii + 2];
          if (par1 == 99) {
            nrpnH[ch] = par2;  //Save nrpn high byte (7 bits)
          }
          if (par1 == 98) {
            nrpnL[ch] = par2;  //Save nrpn low byte (7 bits)
          }
          if (par1 == 6) {  //NRPN value commmand

            uint16_t n = nrpnH[ch] * 128;
            n = n + (nrpnL[ch] & 0x7f);  //Recover 14 bit NRPN number


            //If it's a swell controller value store the value for future use as remote indicators
            if (ch == chan && n >= nrpnOffset && n < nrpnOffset + 4) {
              rxswell[n - nrpnOffset] = par2;  //Store received contoller value
            }

            if (debug == true) Serial.printf("RX NPRN\t\t%d\t%d\t%d\n\r", ch, n, par2);
            if (n > nrpnOffset + 3 && n < nrpnOffset + 36) {
              //Uodate illuminated pistons if the channel matches
              if (ch == chan) {
                if (par2 == 0) {
                  bitWrite(ledstat, n - nrpnOffset - 4, 1);
                } else {
                  bitWrite(ledstat, n - nrpnOffset - 4, 0);
                }
              }
            }
            if (n >= nrpnOffset && n < nrpnOffset + 36) {
              //Send controller update to I2C Slaves
              for (int iii = 0; iii < i2cmax; iii++) {
                if (i2cchan[iii] == ch) {
                  if (debug == true) Serial.printf(">> I2C\t\t0x%x\t%d\t%d\n\r", i2cfile[iii], n, par2);
                  Wire.beginTransmission(i2cfile[iii]);
                  Wire.write(16 + n - nrpnOffset);
                  Wire.write(par2);
                  Wire.endTransmission();  //stop transmitting
                }
              }
            }
          }
        }
      }
    }
  }
}

//*********************************************************************************************
//****                     Callbacks for USBMidi received messages                         ****
//*********************************************************************************************
//Handle Note on and off are place holders and are not neaded
//currently we are only interested in CC messages so we can illuminate the piston LEDs
void handleNoteOn(byte ch, byte par1, byte par2) {
  // Log when a note on is received.
  if (debug) Serial.printf("USB RX  On: \t%d\t%d\t%d\r\n", ch, par1, par2);
}

void handleNoteOff(byte ch, byte par1, byte par2) {
  // Log when a note off is received.
  if (debug) Serial.printf("USB RX Off: \t%d\t%d\t%d\r\n", ch, par1, par2);
}

void handleCC(byte ch, byte par1, byte par2) {
  // Log when a Continous Control is received.
  if (debug) Serial.printf("USB RX  CC: \t%d\t%d\t%d\r\n", ch, par1, par2);

  if (par1 == 99) {
    nrpnH[ch] = par2;  //Save nrpn high byte (7 bits)
  }
  if (par1 == 98) {
    nrpnL[ch] = par2;  //Save nrpn low byte (7 bits)
  }
  if (par1 == 6) {  //NRPN value commmand

    uint16_t n = nrpnH[ch] * 128;
    n = n + (nrpnL[ch] & 0x7f);  //Recover 14 bit NRPN number


    //If it's a swell controller value store the value for future use as remote indicators
    if (ch == chan && n >= nrpnOffset && n < nrpnOffset + 4) {
      rxswell[n - nrpnOffset] = par2;  //Store received contoller value
    }

    if (debug == true) Serial.printf("RX NPRN\t\t%d\t%d\t%d\n\r", ch, n, par2);
    if (n >= nrpnOffset && n < nrpnOffset + 36) {
      //Send controller update to I2C Slaves
      for (int iii = 0; iii < i2cmax; iii++) {
        if (i2cchan[iii] == ch) {
          if (debug == true) Serial.printf("\t\t0x%x\t%d\t%d\n\r", i2cfile[iii], (n - nrpnOffset), par2);
          Wire.beginTransmission(i2cfile[iii]);
          Wire.write(16 + n - nrpnOffset);
          Wire.write(par2);
          Wire.endTransmission();  //stop transmitting
        }
      }
    }
  }
}
