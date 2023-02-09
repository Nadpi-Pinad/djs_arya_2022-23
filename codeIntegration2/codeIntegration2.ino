/***********************************************Tasks to be done********************************************/
//Storing values from BMP280 in EEPROM
//Finding the tilt angles and the orientation of the CanSat
//Finding coordinates of the CanSat using GPS
//Storing values in SDcard using Teensy4.1
//Measuring UTC time using Teensy 4.1
//voltage divider using Teensy4.1   

/********************************************************Library********************************************/
#include <EEPROM.h> //EEPROM library
#include <Adafruit_BMP280.h> //adafruit BMP280 library
#include <TimeLib.h> //UTC time library
#include <SD.h> //SD card library0
#include <SPI.h> //SPI interface
#include <Adafruit_GPS.h>
#include <MPU9250_WE.h>
#include <Wire.h>

/********************************************************Variable********************************************/
float referenceAltitude,actualAltitude,realAltitude,tiltx,tilty,orientation;
int flag,s,sats;
double temperature,galtitude,pressure;
float i,t;
int period=1000;
char buffer[40];
unsigned long time_now=0;
char storedArray[30];
const int chipSelect = BUILTIN_SDCARD;
int j;
char array[j];
uint32_t timer = millis();
int Actualtime,UTCtime,UTCtime1,UTCtime2;
const float referenceVolts = 9;
const float R1 = 1000; 
const float R2 = 4700;
const float resistorFactor = 1023.0 * (R2/(R1 + R2));
const int batteryPin = A0;
int val = analogRead(batteryPin);   

/********************************************************Object********************************************/
Adafruit_BMP280 bmp; // I2C Interface
File dataFile;
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
Adafruit_GPS GPS(&GPSSerial);

/********************************************************Define********************************************/
#define MPU9250_ADDR 0x68
#define GPSSerial Serial
#define GPSECHO false
#define TIME_HEADER  "T"   // Header tag for serial time sync message
  
/********************************************************User Defined Functions********************************************/
void bmp280();
void tpa();
void mpu_init();
void mpu_tilt();
void GPS_setup();
void gps_init();
void gps_loop();
void sdcard(); //SDcard initialization
void Program(); //Function for saving the program in SDcard and reading the saved program in SDcard
void rtc_set();
void rtc_loop();
void rtc_printDigits(int digits);  // utility function for digital clock display
void rtc_digitalClockDisplay();   // digital clock display of the time
unsigned long processSyncMessage() ;
void VD();

/********************************************************Setup********************************************/
void setup() {
  bmp280();
  mpu_init();
  GPS_setup();
  gps_init();
  sdcard();
  rtc_set();
}

/********************************************************Loop********************************************/
void loop() 
{
  if(millis() > time_now+period)
  {
  buffer_fun();
  time_now= millis();
}

/********************************************************User Defined Function Description********************************************/

//_____________________________BMP280 EEPROM________________________________________
void bmp280(){
  Serial.begin(9600);
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

  else{
    referenceAltitude = EEPROM.read(20);
   }

  Serial.print("The reference Altitude is : ");
  Serial.println(referenceAltitude);
 }

void tpa()
{
    temperature=bmp.readTemperature();
    pressure=bmp.readPressure()/10; //displaying the Pressure in kPa, you can change the unit
    actualAltitude=(bmp.readAltitude());
    realAltitude=actualAltitude-referenceAltitude;
    //Serial.println(realAltitude);
  
}

//_____________________________Tilt angles and orientation using MPU9250________________________________________
void mpu_init() {
Serial.begin(9600);
Wire.begin();
if(!myMPU9250.init()){
Serial.println("MPU9250 does not respond");
}
else{
Serial.println("MPU9250 is connected");
}

Serial.println("Position you MPU9250 flat and don’t move it – calibrating...");
delay(1000);
myMPU9250.autoOffsets();
Serial.println("Done!");
/myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);

myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);

myMPU9250.enableAccDLPF(true);

myMPU9250.setAccDLPF(MPU9250_DLPF_6);
}

void mpu_tilt() {
xyzFloat gValue = myMPU9250.getGValues();
xyzFloat angle = myMPU9250.getAngles();
 
  tiltx =angle.x;
  tilty = angle.y;
 orientation = myMPU9250.getOrientationAsString(); 
}     

//__________________________________________GPS________________________________________
void GPS_setup()
{
 GPS.begin(9600);
 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
 GPSSerial.println(PMTK_Q_RELEASE);
}
void gps_init()
{
 Serial.begin(115200);
 Serial.println("Adafruit GPS library basic parsing test!");
 GPS.begin(9600);
 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
 delay(1000);
 GPSSerial.println(PMTK_Q_RELEASE); 
}
void gps_loop ()
{
 char c = GPS.read();
 if (GPSECHO)
 if (c) Serial.print(c);
 if (GPS.newNMEAreceived()) 
 {
 Serial.print(GPS.lastNMEA()); 
 if (!GPS.parse(GPS.lastNMEA())) 
 return; 
 }
 if (millis() - timer > 2000) 
 {
 timer = millis(); 
 //Serial.print("\nTime: ");
 if (GPS.hour < 10) { 
   Serial.print('0'); }
 Serial.print(GPS.hour, DEC); 

 if (GPS.minute < 10) { 
   Serial.print('0'); 
   }
 Serial.print(GPS.minute, DEC); 
 if (GPS.seconds < 10) { 
   Serial.print('0'); 
   }
 Serial.print(GPS.seconds, DEC);
 if (GPS.milliseconds < 10) 
 Serial.print("00");
 } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100)
 {
 Serial.print("0");
 }
 Serial.println(GPS.milliseconds);

 Serial.print(GPS.latitude, 4); 
 Serial.print(GPS.lat);
 //Serial.print(", ");
 Serial.print(GPS.longitude, 4); 
 Serial.println(GPS.lon);
 //Serial.print("Altitude: "); 
 gAltitude = GPS.altitude);
 //Serial.print("Satellites: "); 
 sats = (int)GPS.satellites;
 }
 }
}

//__________________________________________SDcard________________________________________
void sdcard(){
  if (!SD.begin(chipSelect)) {
  Serial.println("Initialization failed!");
   return;
   }
 Serial.println("Initialization done!");
}

void Program(){
  //For saving data in SDcard
  dataFile = SD.open("sdcard_t.csv", FILE_WRITE);
  sprintf(storedArray,"My,name,is,Janhvi,");
  for (i = 0; i < 5; i++){
        dataFile.print(storedArray);
        dataFile.println();
  }
  //dataFile.println();
  dataFile.close();
  
// load program from card
  dataFile = SD.open("sd_card.csv");
  dataFile.read(&array[j],array[j]);
  dataFile.close(); 
}    

//__________________________________________RTC________________________________________
void rtc_set(){
  setSyncProvider(getTeensy3Time);

  Serial.begin(9600);
  while (!Serial);
  delay(100);
  if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("According to UTC Time");
  }
}

void rtc_loop()
{ // digital clock display of the time
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
 float volts = (val / resistorFactor) * referenceVolts; // calculate the ratio
}

/**************************Buffer function*****************************/
void buffer_fun()
{
   tpa();
  mpu_tilt();
  gps_loop();  
  Program();
  rtc_loop();
  VD(); 
sprintf(buffer,"1033, %d:%d:%d, %d, %d, %d, %d ",UTCtime1, UTCtime2, s,realAltitude,temperature, volts, )  
}