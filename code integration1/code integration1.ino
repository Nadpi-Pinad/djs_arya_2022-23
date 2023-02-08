/***********************************************Tasks to be done********************************************/
//Storing values from BMP280 in EEPROM
//Measuring UTC time using Teensy 4.1
//Storing values in SDcard using Teensy4.1
//Finding coordinates of the CanSat using GPS
//Finding the tilt angles and the orientation of the CanSat

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
float referenceAltitude,actualAltitude,realAltitude;
int flag;
float i,t;
int period=1000;
char buffer[40];
unsigned long time_now=0;
char storedArray[30];
const int chipSelect = BUILTIN_SDCARD;
int i;
char array[i];
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
void sdcard(); //SDcard initialization
void Program(); //Function for saving the program in SDcard and reading the saved program in SDcard
void init();
void GPS_setup();
void gps_init();
void gps_loop();
void tilt();
void rtc_set();
void rtc_loop();
void rtc_printDigits(int digits);  // utility function for digital clock display
void rtc_digitalClockDisplay();   // digital clock display of the time
unsigned long processSyncMessage() ;
void VD();

/********************************************************Setup********************************************/
void setup() {
bmp280();
sdcard();
init();
gps_init();
GPS_setup();
rtc_set();
}

/********************************************************Loop********************************************/
void loop() 
{
  tpa();
  Program();
  gps_loop();  
  tilt();
  rtc_loop();
  VD();

}

/********************************************************User Defined Function Description********************************************/
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

void tpa(){
  if(millis() > time_now+period)
  {
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure()/100); //displaying the Pressure in hPa, you can change the unit
    Serial.println(" hPa");
    
    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1019.66)); //The "1019.66" is the pressure(hPa) at sea level in day in your region
    Serial.println(" m");                    //If you don't know it, modify it until you get your current altitude
    actualAltitude=(bmp.readAltitude());
    realAltitude=actualAltitude-referenceAltitude;
    Serial.println(realAltitude);
  }   
}
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
  dataFile.read(&array[i],array[i]);
  dataFile.close(); 
}    
void init() {
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
void tilt() {
xyzFloat gValue = myMPU9250.getGValues();
xyzFloat angle = myMPU9250.getAngles();
if(millis() > time_now+period){
Serial.print("TILT_X = ");
Serial.print(angle.x);
Serial.print(" | TILT_Y = ");
Serial.println(angle.y);
Serial.print("Orientation of the module: ");
Serial.println(myMPU9250.getOrientationAsString());
Serial.println();  
}
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
 Serial.print("\nTime: ");
 if (GPS.hour < 10) { Serial.print('0'); }
 Serial.print(GPS.hour, DEC); Serial.print(':');
 if (GPS.minute < 10) { Serial.print('0'); }
 Serial.print(GPS.minute, DEC); Serial.print(':');
 if (GPS.seconds < 10) { Serial.print('0'); }
 Serial.print(GPS.seconds, DEC); Serial.print('.');
 if (GPS.milliseconds < 10) {
 Serial.print("00");
 } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
 Serial.print("0");
 }
 Serial.println(GPS.milliseconds);
 Serial.print("Date: ");
 Serial.print(GPS.day, DEC); Serial.print('/');
 Serial.print(GPS.month, DEC); Serial.print("/20");
 Serial.println(GPS.year, DEC);
 Serial.print("Fix: "); Serial.print((int)GPS.fix);
 Serial.print(" Quality: "); Serial.println((int)GPS.fixquality);
 if (GPS.fix) {
 Serial.println("Location: ");
 Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
 Serial.print(", ");
 Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
 Serial.print("Altitude: "); Serial.println(GPS.altitude);
 Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
 }
 }
}
void GPS_setup()
{
 GPS.begin(9600);
 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
 GPSSerial.println(PMTK_Q_RELEASE);
}
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
void rtc_loop() {
  if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }
  rtc_digitalClockDisplay();  
  delay(1000);
}
void rtc_printDigits(int digits)
{ // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
void rtc_digitalClockDisplay()
{ // digital clock display of the time
  Actualtime=hour()*60+minute();//conversion of hour to min
  UTCtime=Actualtime-(5*60+30);
  UTCtime1=UTCtime/60;//conversion of min to hour
  UTCtime2=UTCtime%60;//modulus formation
   Serial.print(UTCtime1);
   Serial.print(":");
   Serial.print(UTCtime2);
   Serial.print(":");
   Serial.println(second());
}
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
unsigned long processSyncMessage() 
 {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 
  if(Serial.find(TIME_HEADER)) 
  {
     pctime = Serial.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) 
     { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}
void VD()
{
 float volts = (val / resistorFactor) * referenceVolts; // calculate the ratio
 Serial.println(volts); // print the value in volts
}