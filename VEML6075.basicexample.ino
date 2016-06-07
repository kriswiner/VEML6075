/* VEML6075_t3 Basic Example Code
 by: Kris Winer
 date: June 6, 2016
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic VEML6074 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled UVA/B intensity data out. Sketch runs on the 3.3 V Teensy 3.1.
 
 From the data sheet: (http://www.vishay.com/docs/84304/veml6075.pdf)
 
 The VEML6075 senses UVA and UVB light and incorporates photodiode,  amplifiers,  and  analog  /  digital  circuits  into  a  
single  chip  using  a  CMOS  process.  When  the  UV  sensor  is  applied, it is able to detect 
UVA and UVB intensity to provide a  measure  of  the  signal  strength  as  well  as  allowing  for  UVI  
measurement. The VEML6075 provides excellent temperature compensation capability  for  keeping  the  output  stable  under  changing  
temperature. VEML6075’s functionality is easily operated via the  simple  command  format  of  I2C  (SMBus  compatible)  interface  protocol.  
VEML6075’s  operating  voltage  ranges  from 1.7 V to 3.6 V. VEML6075 is packaged in a lead (Pb)-free 4  pin  OPLGA  package  which  offers  
the  best  market-proven  reliability.

 SDA and SCL  have external pull-up resistors (to 3.3V).
 2K2 resistors are on the VEML6070 breakout board.
 
 Hardware setup:
 VEML6075 Breakout ------ Teensy 3.1
 VDD ---------------------- 3.3V or any digital pin i.e., digitalWrite(HIGH)
 SDA -----------------------pin 17 or 18
 SCL -----------------------pin 16 or 19
 GND ---------------------- GND or any digital pin i.e., digitalWrite(LOW)
 
*/
  
#include <i2c_t3.h>

////////////////////////////
// VEML6075 Command Codes //
////////////////////////////
#define  VEML6075_UV_CONF	        0x00 // command codes
#define  VEML6075_UVA_DATA   		  0x07  // 2 bytes
#define  VEML6075_UVDUMMY_DATA    0x08  
#define  VEML6075_UVB_DATA        0x09  
#define  VEML6075_UVCOMP1_DATA    0x0A  
#define  VEML6075_UVCOMP2_DATA    0x0B  
#define  VEML6075_UV_ID           0x0C  // should retrn 0x26

#define ACoef 3.33
#define BCoef 2.5
#define CCoef 3.66
#define DCoef 2.75
#define UVAresponsivity  0.0011
#define UVBresponsivity  0.00125

#define VEML6075_ADDRESS          0x10 // same as VEML6075, so can't easily combine them

#define SerialDebug true  // set to true to get Serial output for debugging

// Pin definitions
int myLed  = 13;                      
uint16_t count = 0;

enum IT {
  IT_50 = 0,  //   50 ms
  IT_100,     //  100 ms
  IT_200,     //  200 ms
  IT_400,     //  400 ms
  IT_800      //  800 ms
};

// Specify VEML6075 Integration time
uint8_t IT = IT_100;
uint8_t ITime = 100;  // milliseconds
int16_t UVData[5] = {0, 0, 0, 0, 0}; // UVA, Dummy, UVB, UVComp1, UVComp2
float UVASensitivity = 0.93/((float) (IT + 1)); // UVA light sensitivity increases with integration time
float UVBSensitivity = 2.10/((float) (IT + 1)); // UVB light sensitivity increases with integration time
float UVAComp, UVBComp, UVIndex;

void setup()
{
 // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(4000);
  Serial.begin(38400);
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  I2Cscan();

  uint16_t deviceID = getVEML6075ID();
  Serial.print("VEML6075 Device ID = 0x"); Serial.println(deviceID, HEX);
  Serial.println(" ");
  delay(1000);
  
  if(deviceID == 0x26)
  {
  enableVEML6075(); // initalize sensor
 
  delay(150);
  
  digitalWrite(myLed, LOW);
  }
  else
  {
    Serial.print("Could not connect to VEML6075: 0x"); Serial.println(deviceID, HEX);
    while(1) ; // Loop forever if communication doesn't happen
 }
 
} // end setup

void loop()
{  
  getUVdata(UVData);
  Serial.print("UVA raw counts = ");        Serial.println(UVData[0]);
  Serial.print("UV Dummy raw counts = ");   Serial.println(UVData[1]);
  Serial.print("UVB raw counts = ");        Serial.println(UVData[2]);
  Serial.print("UV Comp1 raw counts = ");   Serial.println(UVData[3]);
  Serial.print("UV Comp2 raw counts = ");   Serial.println(UVData[4]);
  Serial.println("  ");

  Serial.print("UVA intensity = "); Serial.print(((float)UVData[0])/UVASensitivity, 2); Serial.println(" microWatts/cm^2");
  Serial.print("UVB intensity = "); Serial.print(((float)UVData[2])/UVBSensitivity, 2); Serial.println(" microWatts/cm^2");

  // Calculate the UV Index, valid in open air not behind glass!
  UVAComp = (UVData[0] - UVData[1]) - ACoef*(UVData[3] - UVData[1]) - BCoef*(UVData[4] - UVData[1]);
  UVBComp = (UVData[2] - UVData[1]) - CCoef*(UVData[3] - UVData[1]) - DCoef*(UVData[4] - UVData[1]);
  UVIndex = (  (UVBComp*UVBresponsivity) +  (UVAComp*UVAresponsivity)  )/2.;
  
  Serial.print("UV Index = "); Serial.println(UVIndex, 2); 
  if(UVIndex <= 2) Serial.println("Strength of UV Irradiance is LOW");
  if(UVIndex > 2 && UVIndex <= 6) Serial.println("Strength of UV Irradiance is MODERATE");
  if(UVIndex > 6 && UVIndex <= 8) Serial.println("Strength of UV Irradiance is HIGH");
  if(UVIndex > 8 && UVIndex <= 10) Serial.println("Strength of UV Irradiance is VERY HIGH");
  if(UVIndex > 10) Serial.println("Strength of UV Irradiance is EXTREME");
  Serial.println("  ");

  digitalWrite(myLed, !digitalRead(myLed));
  delay(ITime+20);
}

//===================================================================================================================
//====== Set of useful function to access VEML6075 UV data
//===================================================================================================================

uint16_t getVEML6075ID()
{
    uint8_t rawData[2] = {0, 0};
    Wire.beginTransmission(VEML6075_ADDRESS);
    Wire.write(0x0C);        // Command code for reading VEML6075 ID
    Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive

    Wire.requestFrom(VEML6075_ADDRESS, 2);  // Read two bytes from slave register address 
    uint8_t i = 0;
    while (Wire.available()) 
    {
        rawData[i++] = Wire.read();       // Put read results in the Rx buffer
    }     
    Wire.endTransmission();
    return ((uint16_t) rawData[1] << 8) | rawData[0];
}



uint16_t getUVdata(int16_t * destination)
{
    for (int j = 0; j < 5; j++)
    {
    uint8_t rawData[2] = {0, 0};
    Wire.beginTransmission(VEML6075_ADDRESS);
    Wire.write(VEML6075_UVA_DATA + j);        // Command code for reading UV data channels in sequence
    Wire.endTransmission(I2C_NOSTOP);         // Send the Tx buffer, but send a restart to keep connection alive

    Wire.requestFrom(VEML6075_ADDRESS, 2);    // Read two bytes from slave register address 
    uint8_t i = 0;
    while (Wire.available()) 
    {
        rawData[i++] = Wire.read();       // Put read results in the Rx buffer
    }     
    Wire.endTransmission();
    destination[j] = ((int16_t) rawData[1] << 8) | rawData[0];
    }
 
}

void enableVEML6075()
{
  Wire.beginTransmission(VEML6075_ADDRESS);
  Wire.write(VEML6075_UV_CONF); // Command code for configuration register
  Wire.write(IT << 4); // Bit 3 must be 0, bit 0 is 0 for run and 1 for shutdown, LS Byte
  Wire.write(0x00); // MS Byte
  Wire.endTransmission();
}


// I2C scan function

void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
}
