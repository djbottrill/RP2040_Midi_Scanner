#pragma once
//GPIO configuration data for RP2040 V7a scanner PCB
#define ledOn  1
#define ledOff 0

#define I2CSDA 0              //I2C BUS pins
#define I2CSCL 1

#define i2cReset 22           //Bus Reset in Master Mode

#define maxSwells 3           //Maximum number of swell inputs
#define swell0 26             //Swell controller inputs
#define swell1 27
#define swell2 28
#define swell3 28

#define col0 2
#define col1 3
#define col2 4
#define col3 5
#define col4 6
#define col5 7
#define col6 8
#define col7 9

#define row0 10
#define row1 11
#define row2 12
#define row3 13
#define row4 14
#define row5 15
#define row6 20
#define row7 21


uint16_t pxlte[] = {          //GPIO sequence order for illuminated pistons
  col0, col1, col2, col3,
  col4, col5, col6, col7,
  row0, row1, row2, row3,
  row4, row5, row6, row7
};

//SPI conectors for Wiznet W5500 Lite Ethernet adapter
#define SS_W5500 17           //SPI Slave Select
/*
MISO is GPIO     16
SCLK is GPIO     18
MOSI is GPIO     19
*/
