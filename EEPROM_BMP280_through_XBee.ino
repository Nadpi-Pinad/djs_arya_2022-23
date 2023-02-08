/**************Tasks to be done***************/
//Storing values from BMP280 in EEPROM

/**************Library***************/
#include <EEPROM.h> //EEPROM library
#include <Adafruit_BMP280.h> //adafruit BMP280 library
#include <SoftwareSerial.h>

/**************Variable***************/
float referenceAltitude,actualAltitude,realAltitude;
int flag;
float i,t;
int temp;

Adafruit_BMP280 bmp; // I2C Interface
SoftwareSerial xbee = SoftwareSerial(0,1); // XBee Interface 

/**************Define***************/

/**************UserDefine Function declaration***************/
void eepromclear();
void reading();
void setup2();
void loop2();


/**************Setup***************/
void setup() 
{
Serial.begin(9600);
pinMode(0,INPUT);
pinMode(1,OUTPUT);
Serial.println(F("BMP280 test"));
if (!bmp.begin(0x76))
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
   
 flag=EEPROM.read(25);
 if(flag==0)
 {
  referenceAltitude = bmp.readAltitude();
  EEPROM.write(20,referenceAltitude);
  flag = 9;
  EEPROM.write(25,flag);
 }
 else
 {
  referenceAltitude = EEPROM.read(20);
 }
 Serial.print("The reference Altitude is : ");
 Serial.println(referenceAltitude);
}

/**************Loop***************/
void loop() 
{
  if(xbee.available()>0)
  {
    temp=xbee.read();
    xbee.print(temp);
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure()/100); //displaying the Pressure in hPa, you can change the unit
    Serial.println(" hPa");
    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1019.66)); //The "1019.66" is the pressure(hPa) at sea level in day in your region
    Serial.println(" m");                    //If you don't know it, modify it until you get your current altitude
    actualAltitude=(bmp.readAltitude());
    realAltitude=actualAltitude-referenceAltitude;
    Serial.println(realAltitude);
    delay(2000);
    eepromclear();
  }
}

void eepromclear()
{
  setup2();
  loop2();
}
  
void setup2()
{
  pinMode(13, OUTPUT);
  for (int i = 0 ; i < EEPROM.length() ; i++) 
  {
    EEPROM.write(i, 0);
  }
  digitalWrite(13, HIGH);
}

void loop2() 
{
}

    /*Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure()/100); //displaying the Pressure in hPa, you can change the unit
    Serial.println(" hPa");
    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1019.66)); //The "1019.66" is the pressure(hPa) at sea level in day in your region
    Serial.println(" m");                    //If you don't know it, modify it until you get your current altitude
    actualAltitude=(bmp.readAltitude());
    realAltitude=actualAltitude-referenceAltitude;
    Serial.println(realAltitude);
    delay(2000);*/
