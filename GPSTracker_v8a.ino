//GPS Tracker (Arduino-Compatible)

//David Eldridge 2017
//david.eldridge1987@gmail.com

//Licensing Blah Blah
//Built to track Pedal Prix bikes in the Australian International Pedal Prix Series

/* Acknowledgements
   iforce2d www.iforce2d.net, programming and reading the GPS
   maniacbug http://maniacbug.wordpress.com, ATMega1284 Platform for Arduino
   Andrew Rapp http://rapplogic.blogspot.com.au/ https://github.com/andrewrapp, Arduino xBee API Library
   Limor Fried (ladyada) https://github.com/adafruit www.ladyada.net https://www.adafruit.com/, MCP23017 Library


*/

/* Equipment List
   ATMega1284 with Arduino Bootloader
   uBlox GPS Module Neo 7 Series or higher
   xBee.........
   MCP23017 I2C Port Expander
   YwRobot Arduino LCM1602 16x2 LCD Screen
*/


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <XBee.h>
#include "Adafruit_MCP23017.h"

HardwareSerial & xBeeSerial = Serial;
HardwareSerial & gpsSerial = Serial1;

//Variables
//Change these as needed!
const int mcpAddress  = 0; //MCP23017 Address, see table below to select correct address
const int riderChange = 1; //Corresponds to pin 21 on the MCP23017
const int waterPit    = 2; //Corresponds to pin 22 on the MCP23017
const int notifyLED   = 3; //Corresponds to pin 23 on the MCP23017

const int nodeID      = 101; //GPS Unit ID Number, use this number in the program

float bikeLat;
float bikeLon;
float bikeSpeed;
float battVolt;
float battCell1;
float battCell2;
float battCell3;
float battVoltTmp;
float battVoltage;
int gpsLock;
int headlightStatus;
int bikeStatus;
int msgAck;
int loopCount;

//16x2 I2C LCD Parameters
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

/* Addresses for the MCP23017, low->GND, high->5v
      Library                               Chip  i2c
      Address                               Addr  Addr
      addr 0 = A2 low   A1 low    A0 low    000   0x20
      addr 1 = A2 low   A1 low    A0 high   001   0x21
      addr 2 = A2 low   A1 high   A0 low    010   0x22
      addr 3 = A2 low   A1 high   A0 high   011   0x23
      addr 4 = A2 high  A1 low    A0 low    100   0x24
      addr 5 = A2 high  A1 low    A0 high   101   0x25
      addr 6 = A2 high  A1 high   A0 low    110   0x26
      addr 7 = A2 high  A1 high   A0 high   111   0x27
*/
Adafruit_MCP23017 mcpDisplayBoard;

//xBee Settings
XBee                xbee;
uint8_t             xbeeTxPayload[ 19 ];
XBeeAddress64       addr64( 0x0013a200, 0x414E4D46 );
ZBTxRequest         zbTx( addr64, xbeeTxPayload, sizeof(xbeeTxPayload) );
ZBTxStatusResponse  txStatus;
XBeeResponse        response;
ZBRxResponse        rx;
ModemStatusResponse msr;
uint8_t             xbeeRxPayload[ 17 ];

union{
  char rxString[];
  byte a[16];
} rxData;

// GPS Programming and Setup
const char UBLOXGPS_PROG[] PROGMEM = {
  //Disable NMEA Messages
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, // GxGGA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40, // GxRMC off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off

  //Disable UBX Messages
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC, //NAV-PVT off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, //NAV-POSLLH off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, //NAV-STATUS off

  //Enable Wanted Messages
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1, //NAV-PVT on
  //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE, //NAV-POSLLH on
  //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x14, 0xC5, //NAV-STATUS on

  //Set the Baud Rate

  //Set the Update Rate
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12, //(10Hz)
  //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, //(5Hz)
  //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39, //(1Hz)
};

const unsigned char UBX_HEADER[] = {0xB5, 0X62};

typedef struct {
  byte b0: 1;
  byte b1: 1;
  byte b2: 1;
  byte b3: 1;
  byte b4: 1;
  byte b5: 1;
  byte b6: 1;
  byte b7: 1;
} eightBitField;

typedef struct {
  byte b0: 1;
  byte b1: 1;
  byte b2: 1;
  byte b3: 1;
  byte b4: 1;
  byte b5: 1;
  byte b6: 1;
  byte b7: 1;
  byte b8: 1;
  byte b9: 1;
  byte b10: 1;
  byte b11: 1;
  byte b12: 1;
  byte b13: 1;
  byte b14: 1;
  byte b15: 1;
} sixteenBitField;

struct NAV_PVT {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  unsigned short year;
  unsigned char month;
  unsigned char day;
  unsigned char hour;
  unsigned char minute;
  unsigned char second;
  eightBitField valid;
  unsigned long tAcc;
  long nano;
  unsigned char fixType;
  eightBitField flags;
  unsigned char reserved1;
  unsigned char numSV;
  long longitude;
  long latitude;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
  long velN;
  long velE;
  long delD;
  long gSpeed;
  long heading;
  unsigned long sAcc;
  unsigned long headingAcc;
  unsigned short pDOP;
  sixteenBitField reserved2;
  unsigned long reserved3;
};

NAV_PVT pvt;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_PVT); i++) {
    CK[0] += ((unsigned char*)(&pvt))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int dataPos = 0;
  static unsigned char checksum[2];
  const int gpsPayloadSize = sizeof(NAV_PVT);

  while (gpsSerial.available()) {
    byte c = gpsSerial.read();
    if (dataPos < 2) {
      if (c == UBX_HEADER[dataPos]) {
        dataPos++;
      }
      else {
        dataPos = 0;
      }
    }
    else {
      if ( (dataPos - 2) < gpsPayloadSize )
        ((unsigned char*)(&pvt))[dataPos - 2] = c;
      dataPos++;
      if ( dataPos == (gpsPayloadSize + 2) ) {
        calcChecksum(checksum);
      }
      else if ( dataPos == (gpsPayloadSize + 3) ) {
        if ( c != checksum[0] )
          dataPos = 0;
      }
      else if ( dataPos == (gpsPayloadSize + 4) ) {
        dataPos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( dataPos > (gpsPayloadSize + 4) ) {
        dataPos = 0;
      }
    }
  }
  return false;
}

void setup() {
  lcdSetup();
  progGPS();
  mcpSetup();
}

//Setup inital parameters for the LCD Screen and Welcome Message
void lcdSetup() {
  lcd.begin(16, 2); //16x2 Characters
  lcd.backlight(); //Backlight ON
  lcd.setCursor(0, 0);
  lcd.print("GPS Tracker");
  lcd.setCursor(0, 1);
  lcd.print("David Eldridge");
  lcd.setCursor(0, 0);
  delay(5000);
}

//Programming the GPS Module
void progGPS() {
  gpsSerial.begin(9600);
  lcd.clear();
  lcd.print("Programming GPS.");

  // Send the configuration to the GPS as set out in UBLOXGPS_PROG
  for (int i = 0; i < sizeof(UBLOXGPS_PROG); i++) {
    //gpsSerial.write(pgm_read_byte(UBLOXGPS_PROG +i));
    delay(5); //simulating a 38400 baud rate (or less), commands are no accepted by the device otherwise.
  }
  lcd.clear();
  lcd.print("Programming GPS");
  lcd.setCursor(0, 1);
  lcd.print("Complete!");
  delay(5000);
  lcd.clear();
}

void mcpSetup() {
  mcpDisplayBoard.begin(mcpAddress);
  mcpDisplayBoard.pinMode(riderChange, INPUT);
  mcpDisplayBoard.pullUp(riderChange, HIGH);
  mcpDisplayBoard.pinMode(waterPit, INPUT);
  mcpDisplayBoard.pullUp(waterPit, HIGH);
  mcpDisplayBoard.pinMode(notifyLED, OUTPUT);
}

void loop() {
  xBeeRx();
  xBeeTx();
}

void xBeeTx() {
  checkSwitches();
  battVoltage = getBattVolt();
  if (processGPS()) {
    bikeLat = pvt.latitude / 10000000.0f;
    bikeLon = pvt.longitude / 10000000.0f;
    bikeSpeed = pvt.gSpeed / 100.0f;
  }
  lcd.setCursor(0, 1);
  lcd.print(bikeSpeed);
  lcd.print( F("km/hr") );

  xbeeTxPayload[0] = nodeID;
  memcpy( &xbeeTxPayload[ 1], &bikeLat    , sizeof(bikeLat)     );
  memcpy( &xbeeTxPayload[ 5], &bikeLon    , sizeof(bikeLon)     );
  memcpy( &xbeeTxPayload[ 9], &bikeSpeed  , sizeof(bikeSpeed)   );
  memcpy( &xbeeTxPayload[13], &battVoltTmp, sizeof(battVoltTmp) );
  xbeeTxPayload[17] = bikeStatus;
  xbeeTxPayload[18] = msgAck;
  xbee.send(zbTx);
  msgAck = 0;
}

//The following function reads the incoming packet and if the length of the packet is 17 bytes then processes it
//memcpy is memory copy and its structure is destination array, source array, size of the source array.
//It will also print the string recieved by the xbee to the screen, maximum of 16 characters.
void xBeeRx() {
  xbee.readPacket();
  if (loopCount == 600){
     mcpDisplayBoard.digitalWrite(notifyLED, LOW);
     loopCount = 0;
  }
  if (xbee.getResponse().isAvailable()) {
    if (rx.getDataLength() == 17) {
      memcpy ( &xbeeRxPayload[0], rx.getData(),       sizeof(rx.getData())        );
      memcpy ( &rxData.a[0],      &xbeeRxPayload[1],  sizeof(&xbeeRxPayload - 1)  );
      int ID = rx.getData()[0];
      if (ID == nodeID) {
        mcpDisplayBoard.digitalWrite(notifyLED, HIGH);
        lcd.setCursor(0, 0);
        lcd.print(rxData.rxString);
        msgAck = 1;
      }
    }
  }
  loopCount++;
}

void checkSwitches() {
  if (mcpDisplayBoard.digitalRead(riderChange) == HIGH && mcpDisplayBoard.digitalRead(waterPit) == HIGH) {
    bikeStatus = 0;
    //lcd.print("A-OK!");
  }
  else if (mcpDisplayBoard.digitalRead(riderChange) == LOW && mcpDisplayBoard.digitalRead(waterPit) == HIGH) {
    bikeStatus = 1; //Rider Change
    //lcd.print("Rider CHANGE!");
  }
  else if (mcpDisplayBoard.digitalRead(riderChange) == HIGH && mcpDisplayBoard.digitalRead(waterPit) == LOW) {
    bikeStatus = 2; //Coming in for Water
    //lcd.print("WATER!");
  }
  else if (mcpDisplayBoard.digitalRead(riderChange) == LOW && mcpDisplayBoard.digitalRead(waterPit) == LOW) {
    bikeStatus = 3; //Big Problems
  }
}

//Read the battery voltage and calculate the lowest Value (Using a 3S)
float getBattVolt() {
  battCell1 = ((analogRead(A0) / 1024.00) * 5.00);
  battCell2 = (((analogRead(A1) / 1024.00) * 5.00) * 2) - battCell1;
  battCell3 = (((analogRead(A2) / 1024.00) * 5.00) * 3) - battCell2;

  //Compare each of the cell voltages and determine which is the lowest to transmit back
  battVoltTmp = battCell1;
  if (battVoltTmp > battCell2) {
    battVoltTmp = battCell2;
  }
  else if (battVoltTmp > battCell3) {
    battVoltTmp = battCell3;
  }
  return battVoltTmp;
}

