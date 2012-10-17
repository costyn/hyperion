/*
Arduino code used on Hyperion flight computer 
Payload consisted of:
 * Seeedstudio stalker v328
 * Radiometrix NTX2 10mW 434.075Mhz
 * GPSbee (Ublox 5) GPS
 * 2x DS18b20 temp sensors
 * BMP085 barometer

Code by James Coxon (jacoxon@googlemail.com) based on previous code as well
as Arduino examples

Minor modifications by Costyn van Dongen
*/
#include <TinyGPS.h>
#include <OneWire.h>
#include <stdio.h>
#include <util/crc16.h>
#include <Wire.h>

#include "SD.h"

#define ONE_WIRE_BUS 10
#define NTX2_SPACE_PIN 4
#define NTX2_MARK_PIN 5
#define NTX2_POWER_PIN 6

TinyGPS gps;
OneWire ds(ONE_WIRE_BUS); // DS18x20 Temperature chip i/o One-wire

//Tempsensor variables
byte address0[8] = {0x10, 0x68, 0x10, 0x36, 0x02, 0x08, 0x00, 0x9D};
byte address1[8] = {0x10, 0xA3, 0x32, 0x36, 0x02, 0x08, 0x00, 0x81};

byte address2[8] = {0x28, 0xE8, 0x89, 0xC2, 0x2, 0x0, 0x0, 0xDF}; // placeholder
int temp0 = 0, temp1 = 0, temp2 = 0;

int count = 1, nightloop = 0;
byte navmode = 99;
float flat, flon;
unsigned long date, time, chars, age;

int hour = 0 , minute = 0 , second = 0, oldsecond = 0;
char latbuf[12] = "0", lonbuf[12] = "0", altbuf[12] = "0";
long int ialt = 123;
int numbersats = 99;

// ============ Barometer ================= 

// Barometer
// Calibration values
#define BMP085_ADDRESS 0x77  // I2C address of BMP085
const unsigned char OSS = 0;  // Oversampling Setting

int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

short b_temperature;
char b_temp_s[3];
long b_pressure;
char b_press_s[7];



// ------------------------
// RTTY Functions - from RJHARRISON's AVR Code
void rtty_txstring (char * string)
{
	/* Simple function to sent a char at a time to 
	** rtty_txbyte function. 
	** NB Each char is one byte (8 Bits)
	*/
	char c;
	c = *string++;
	while ( c != '\0')
	{
		rtty_txbyte (c);
		c = *string++;
	}
}

void rtty_txbyte (char c)
{
	/* Simple function to sent each bit of a char to 
	** rtty_txbit function. 
	** NB The bits are sent Least Significant Bit first
	**
	** All chars should be preceded with a 0 and 
	** proceded with a 1. 0 = Start bit; 1 = Stop bit
	**
	** ASCII_BIT = 7 or 8 for ASCII-7 / ASCII-8
	*/
	int i;
	rtty_txbit (0); // Start bit
	// Send bits for for char LSB first	
	for (i=0;i<8;i++)
	{
		if (c & 1) rtty_txbit(1); 
			else rtty_txbit(0);	
		c = c >> 1;
	}
	rtty_txbit (1); // Stop bit
        rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit)
{
		if (bit)
		{
		  // high
                    digitalWrite(NTX2_SPACE_PIN, HIGH);  
                    digitalWrite(NTX2_MARK_PIN, LOW);
                    digitalWrite(13,HIGH); // LED on
		}
		else
		{
		  // low
                    digitalWrite(NTX2_SPACE_PIN, LOW);
                    digitalWrite(NTX2_MARK_PIN, HIGH);
                    digitalWrite(13, LOW); // LED off
		}
		//delayMicroseconds(20500); // 10000 = 100 BAUD 20150
                //delayMicroseconds(20000); // 10000 = 100 BAUD 20150
//                delayMicroseconds(19500); // 50baud ; 10000 = 100 BAUD 20150
                delayMicroseconds(10000); // For 50 Baud uncomment this and the line below. 
                delayMicroseconds(10150); // For some reason you can't do 20150 it just doesn't work.


}

uint16_t gps_CRC16_checksum (char *string)
{
	size_t i;
	uint16_t crc;
	uint8_t c;
 
	crc = 0xFFFF;
 
	// Calculate checksum ignoring the first two $s
	for (i = 2; i < strlen(string); i++)
	{
		c = string[i];
		crc = _crc_xmodem_update (crc, c);
	}
 
	return crc;
}
  
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
	uint8_t b;
	uint8_t ackByteID = 0;
	uint8_t ackPacket[10];
        Serial.flush();
	unsigned long startTime = millis();
 
	// Construct the expected ACK packet    
	ackPacket[0] = 0xB5;	// header
	ackPacket[1] = 0x62;	// header
	ackPacket[2] = 0x05;	// class
	ackPacket[3] = 0x01;	// id
	ackPacket[4] = 0x02;	// length
	ackPacket[5] = 0x00;
	ackPacket[6] = MSG[2];	// ACK class
	ackPacket[7] = MSG[3];	// ACK id
	ackPacket[8] = 0;		// CK_A
	ackPacket[9] = 0;		// CK_B
 
	// Calculate the checksums
	for (uint8_t i=2; i<8; i++) {
		ackPacket[8] = ackPacket[8] + ackPacket[i];
		ackPacket[9] = ackPacket[9] + ackPacket[8];
	}
 
	while (1) {
 
		// Test for success
		if (ackByteID > 9) {
				// All packets in order!
                                navmode = 1;
				return true;
		}
 
		// Timeout if no valid response in 3 seconds
		if (millis() - startTime > 3000) { 
                        navmode = 0;
			return false;
		}
 
		// Make sure data is available to read
		if (Serial.available()) {
			b = Serial.read();

			// Check that bytes arrive in sequence as per expected ACK packet
			if (b == ackPacket[ackByteID]) { 
				ackByteID++;
                                //Serial.print(ackPacket[ackByteID], HEX);
                                //Serial.print(" ");
			} else {
				ackByteID = 0;	// Reset and look again, invalid order
			}
 
		}
	}
}

//Function to poll the NAV5 status of a Ublox GPS module (5/6)
//Sends a UBX command (requires the function sendUBX()) and waits 3 seconds
// for a reply from the module. It then isolates the byte which contains 
// the information regarding the NAV5 mode,
// 0 = Pedestrian mode (default, will not work above 12km)
// 6 = Airborne 1G (works up to 50km altitude)
//Adapted by jcoxon from getUBX_ACK() from the example code on UKHAS wiki
// http://wiki.ukhas.org.uk/guides:falcom_fsa03
boolean checkNAV(){
  uint8_t b, bytePos = 0;
  uint8_t getNAV5[] = { 0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84 }; //Poll NAV5 status
  
  Serial.flush();
  unsigned long startTime = millis();
  sendUBX(getNAV5, sizeof(getNAV5)/sizeof(uint8_t));
  
  while (1) {
    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();
      
      if(bytePos == 8){
        navmode = b;
        return true;
      }
                        
      bytePos++;
    }
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      navmode = 0;
      return false;
    }
  }
}

void setupGPS() {
  //Turning off all GPS NMEA strings apart on the uBlox module
  Serial.println("$PUBX,40,GLL,0,0,0,0*5C");
  Serial.println("$PUBX,40,GGA,0,0,0,0*5A");
  Serial.println("$PUBX,40,GSA,0,0,0,0*4E");
  Serial.println("$PUBX,40,RMC,0,0,0,0*47");
  Serial.println("$PUBX,40,GSV,0,0,0,0*59");
  Serial.println("$PUBX,40,VTG,0,0,0,0*5E");
  
  delay(3000); // Wait for the GPS to process all the previous commands
  
 // Check and set the navigation mode (Airborne, 1G)
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  
  getUBX_ACK(setNav);
  
}

// gets temperature data from onewire sensor network, need to supply byte address, it'll check to see what type of sensor and convert appropriately
int getTempdata(byte sensorAddress[8]) {
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole;
  byte data[12], i, present = 0;
  
  ds.reset();
  ds.select(sensorAddress);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(sensorAddress);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
 LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  
  if (sensorAddress[0] == 0x10) {
    Tc_100 = TReading * 50;    // multiply by (100 * 0.0625) or 6.25
  }
  else { 
    Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25
  }
  
  
  Whole = Tc_100 / 100;  // separate off the whole and fractional portions

  if (SignBit) // If its negative
  {
     Whole = Whole * -1;
  }
  return Whole;
}

unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
//  delay(2 + (3< // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}


char bmp085Read(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  
  return (int) msb<<8 | lsb;
}

short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = 0; // fixme!
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}


void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

void setup()
{
  pinMode(NTX2_SPACE_PIN, OUTPUT); //Radio Tx0
  pinMode(NTX2_MARK_PIN, OUTPUT); //Radio Tx1
  pinMode(NTX2_POWER_PIN, OUTPUT); //Radio En
  digitalWrite(NTX2_POWER_PIN, HIGH);
  Serial.begin(9600);
  
  delay(5000); // We have to wait for a bit for the GPS to boot otherwise the commands get missed
  
  setupGPS();
  
    // Initialize Barometer 
//  Serial.println( "Setting up barometer..." );
//  Wire.begin();
//  bmp085Calibration();
//  Serial.println( "done!" );

}

void loop() { 
    char superbuffer [120];
    char checksum [10];
    int n;
    
    if((count % 10) == 0) {
     if(navmode != 6){
       setupGPS();
       delay(1000);
     }
   }
   
    Serial.println("$PUBX,00*33"); //Poll GPS
    
    while (Serial.available())
    {
      int c = Serial.read();
      if (gps.encode(c))
      {
        //Get Data from GPS library
        //Get Time and split it
        gps.get_datetime(&date, &time, &age);
        hour = (time / 1000000);
        minute = ((time - (hour * 1000000)) / 10000);
        second = ((time - ((hour * 1000000) + (minute * 10000))));
        second = second / 100;
      
      //Get Position
      gps.f_get_position(&flat, &flon);
  
      //convert float to string
      dtostrf(flat, 7, 4, latbuf);
      dtostrf(flon, 7, 4, lonbuf);
      
      //just check that we are putting a space at the front of lonbuf
      if(lonbuf[0] == ' ')
      {
        lonbuf[0] = '+';
      }
      
      // +/- altitude in 
      ialt = (gps.altitude() / 100);   
      itoa(ialt, altbuf, 10);
    }
    }
    
    temp0 = getTempdata(address0);
    temp1 = getTempdata(address1);

//    b_temperature = bmp085GetTemperature(bmp085ReadUT());
//    b_temperature /= 10 ;
//    b_pressure = bmp085GetPressure(bmp085ReadUP());
    dtostrf(b_pressure,1,0,b_press_s);
    numbersats = gps.satellites();
    
    n=sprintf (superbuffer, "$$HYPERION,%d,%02d:%02d:%02d,%s,%s,%ld,%d,%d,%s,%d,%d,%d", count, hour, minute, second, latbuf, lonbuf, ialt, numbersats, navmode, b_press_s, temp0, temp1, b_temperature );
    if (n > -1){
      n = sprintf (superbuffer, "%s*%04X\n", superbuffer, gps_CRC16_checksum(superbuffer));
      rtty_txstring(superbuffer);
      Serial.println(superbuffer); 
    }
    count++;

}
