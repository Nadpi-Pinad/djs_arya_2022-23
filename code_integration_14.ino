/***********************************************Tasks to be done********************************************/
//Storing values from BMP280 in EEPROM
//Finding the tilt angles and the orientation of the CanSat
//Finding coordinates of the CanSat using GPS
//Storing values in SDcard using Teensy4.1
//Measuring UTC time using Teensy 4.1
//voltage divider using Teensy4.1   
//transmission of data using XBee
//Capture the photo/video according to the user input
//Servos tested
//camera are to be tested

/********************************************************Library********************************************/
#include <EEPROM.h> //EEPROM library
#include <Adafruit_BMP280.h> //adafruit BMP280 library
#include <TimeLib.h> //UTC time library
#include <SD.h> //SD card library
#include <SPI.h> //SPI interface
#include <Adafruit_GPS.h>
#include <MPU9250_WE.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <EEPROM.h>
#include <Servo.h>

/********************************************************Variable********************************************/
float referenceAltitude,realAltitude,i,t,volts;
int period=1000,trig=3, flag,s , sats, Actualtime,UTCtime,UTCtime1,UTCtime2,a,time_HS=0,packetCount,packetFlag,EPC;
//char lat_dir,lng_dirn;
int temp; 
uint8_t hr, mins, sec;
unsigned long time_now=0;
char storedArray[20],guiCommand, buffers[150],altitude_resolved[10],lat_val[10],lng_val[10],gAltitude[10],volts_resolved[10],temperature[10],pressure[10],HS = 'N',PC = 'N',Mast = 'N', cam_temporary,xbee;
char tiltx[10], tilty[10];
const int chipSelect = BUILTIN_SDCARD;
uint32_t timer = millis();
const float referenceVolts = 3.3;
const float R1 = 4700; 
const float R2 = 1000;
const float resistorFactor = 1023.0 * (R2/(R1 + R2));
const int batteryPin = A0;
int val;
bool telemetryOnOff=0;
 
/**************************************************Define********************************************/
#define MPU9250_ADDR 0x68
#define GPSSerial Serial2
#define GPSECHO false
#define TIME_HEADER  "T"   // Header tag for serial time sync message                                                                                                                                                                                                                                                                                                                                                             #define chipSelect BUILTIN_SDCARD//FOR ACCESSING THE SD CARD SLOT OF TEENSY 4.1

/********************************************************Object********************************************/
Adafruit_BMP280 bmp; // I2C Interface
File myFile;
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
Adafruit_GPS GPS(&GPSSerial);
TinyGPSPlus gps;
//Servo HServo;
//Servo PServo;
//Servo MServo;
Servo heatShieldServo;
Servo parachuteServo;
Servo flagServo;

/********************************************************User Defined Functions********************************************/
void bmp280();
void tpa();
void bmp_calibration();

void mpu_init();
void mpu_tilt();

void gps_init();
void gps_loop();

void sdcard_1();

void rtc_set();
void rtc_loop();  

void VD();
void buffer_fun();

void packetc_setup();

//void HS_deployed();
//void HS_deployed();
//void HS_deployed();

void xbee_setup();

void video_setup();

void commands();

void servo_setup();

/********************************************************Setup********************************************/
void setup() 
{
  Serial.begin(9600);
  Serial2.begin(9600);
  GPS.begin(9600);
  bmp280();
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  bmp_calibration();
  mpu_init();
  gps_init();
  sdcard_1();
  servo_setup();
  rtc_set();
  xbee_setup();
  packetc_setup();
  video_setup();
}

/********************************************************Loop********************************************/
void loop() 
{
  if(millis() > time_now + period)
  {
   commands();
   buffer_fun();
   time_now=millis();
  }
}

/********************************************************User Defined Function Description********************************************/

//_____________________________BMP280 EEPROM________________________________________

void bmp280()
{
  Serial2.println(F("BMP280 test"));
  if (!bmp.begin(0x76))
   {
    Serial2.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    //while (1);
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

  Serial2.print("The reference Altitude is : ");
  Serial2.println(referenceAltitude);
 }

void tpa()
{
    dtostrf(bmp.readTemperature(),1,1,temperature);
   
    dtostrf((bmp.readPressure()/1000),1,1,pressure); //displaying the Pressure in kPa, you can change the unit
   
    realAltitude=bmp.readAltitude()-referenceAltitude;
    dtostrf(realAltitude,1,1,altitude_resolved);
}

void bmp_calibration(){
  for ( unsigned int cal = 0 ; cal < EEPROM.length() ; cal++ )
    EEPROM.write(cal, 0);
}

//_____________________________Tilt angles and orientation using MPU9250________________________________________
void mpu_init() 
{
 Wire.begin();
 if(!myMPU9250.init())
 {
  Serial2.println("MPU9250 does not respond");
 }
 else
 {
  Serial2.println("MPU9250 is connected");
 }

 Serial2.println("Position your MPU9250 flat and don’t move it – Calibrating...");
 delay(1000);
 myMPU9250.autoOffsets();
 Serial2.println("Done!");
 myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
 myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
 myMPU9250.enableAccDLPF(true);
 myMPU9250.setAccDLPF(MPU9250_DLPF_6);
}

void mpu_tilt() 
{
 xyzFloat angle = myMPU9250.getAngles();
 dtostrf(angle.x ,1,2,tiltx);
 dtostrf(angle.y,1,2,tilty);
}     

//__________________________________________GPS________________________________________

void gps_init()
{
 Serial2.println("Adafruit GPS library basic parsing test!");
 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
 delay(1000);
 GPSSerial.println(PMTK_Q_RELEASE); 
}

void gps_loop ()
{
  hr = gps.time.hour(); 
  mins= gps.time.minute();  
  sec= gps.time.second();  
        
 dtostrf(GPS.latitude,1,4,lat_val);
 //lat_dir= (GPS.lat);
 dtostrf(GPS.longitude,1,4,lng_val);
 //lng_dirn=(GPS.lon);
 dtostrf(GPS.altitude,1,1,gAltitude);
 sats = (int)GPS.satellites;
 }

//__________________________________________SDcard________________________________________
void sdcard_1()
  {
//    while (!Serial2)
//    {}
    if (!SD.begin(chipSelect)) {
      Serial2.println("Initialization failed!");
      return;
    }
    myFile = SD.open("CodeData.csv", FILE_WRITE);
  }
   
//__________________________________________RTC________________________________________
void rtc_set()
{
  setSyncProvider(getTeensy3Time);
  while (!Serial2);
  delay(100);
  if (timeStatus()!= timeSet) 
  {
    Serial2.println("Unable to sync with the RTC");
  } 
  else 
  {
    Serial2.println("According to UTC Time");
  }
}

void rtc_loop()
{ 
  Actualtime=hour()*60+minute();//conversion of hour to min
  UTCtime=Actualtime-(5*60+30);
  UTCtime1=UTCtime/60;//conversion of min to hour
  UTCtime2=UTCtime%60;//modulus formation
  s = second();
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

//__________________________________________Voltage Divider________________________________________

void VD()
{
  val = analogRead(batteryPin); 
 volts = (val / resistorFactor) * referenceVolts; // calculate the ratio
 dtostrf(volts,1,1,volts_resolved);
}

/**************************Buffer function*****************************/

void buffer_fun()
{
 if (telemetryOnOff==1)
  {
   tpa();
   mpu_tilt();
   gps_loop();  
   rtc_loop();
   VD(); 
   EPC = packetCount; 
   packetCount++;
   EEPROM.write(5,packetCount);
   sprintf(buffers,"1033, %d:%d:%d, %d, %s, %s, %s, %s, %02d:%02d:%02d, %s, %s, %s, %d, %s,%s",
           UTCtime1, UTCtime2, s,EPC, altitude_resolved,temperature, volts_resolved, pressure,hr, mins, sec, 
           gAltitude, lat_val, lng_val, sats, tiltx, tilty); 
   //Serial.println(buffers); 
   Serial2.println(buffers);
  }
 myFile = SD.open("CodeData.csv", FILE_WRITE);
 myFile.print(buffers);
 myFile.println();
 myFile.close();
 //Load program
 int j=20;
 char array[j];
 myFile = SD.open("sd_card.csv");
 myFile.read(&array[j],array[j]);
 myFile.close();  
}

//______________________________________SERVO__________________________________________

//void HS_deployed()
//{
//  HServo.attach(9);
//  HServo.write(0);
//  time_HS = millis();
//  if(millis()- time_HS >= 5000)
//  {
//    HServo.write(90);
//    HS = 'P';
//  }
//}
//void PC_deployed()
//{
//  PServo.attach(10);
//  PServo.write(90);
//  PC = 'C';
//}
//void Mast_raised()
//{
//  MServo.attach(11);
//  MServo.write(180);
//  Mast = 'M';
//}

void servo_setup()
{
  heatShieldServo.attach(9);
  parachuteServo.attach(10);
  flagServo.attach(33);
}

//_________________________________________EEPROM PACKET COUNT____________________________________________________________

void packetc_setup() 
{
 packetCount=EEPROM.read(5);
 packetFlag=EEPROM.read(8);
 if(packetFlag==0)
   {
      packetCount=0;
      EEPROM.write(5,packetCount);
      packetFlag=1;
      EEPROM.write(8,flag);
   }
 else
    {
     packetCount=EEPROM.read(5);
     packetCount++;
     EEPROM.write(5,packetCount);
    }
}

//_________________________________________________XBee______________________________________________________

void xbee_setup()
{
  //Serial.println( "Arduino started receiving bytes via XBee" );
   Serial2.println( "Arduino started receiving bytes via XBee" );
}

//_________________________________________________Camera____________________________________________________

void video_setup()
{
  pinMode(trig, OUTPUT);         
  digitalWrite(trig, HIGH); 
}

//_______________________________________XBEEcommunication_______________________________

void commands()
{
  if(Serial2.available()>0)
  {
   xbee = Serial2.read();}
  switch(xbee)
  {
   case '1':
   telemetryOnOff = 1;
   break;

   case '2':
   telemetryOnOff = 0;
   break;

   case 'a':
   heatShieldServo.write(0);
   break;

   case 'A':
   heatShieldServo.write(90);
   break;

   case 'b':
   parachuteServo.write(90);
   break;

   case 'B':
   parachuteServo.write(180);
   break;

   case 'C':
   flagServo.write(90);
   break;

   case 'c':
   flagServo.write(180);
   break;
      
   case '3':
   digitalWrite(trig, LOW);   
   delay(750);               
   digitalWrite(trig, HIGH);    
   xbee=0;
   break;
  }
}
