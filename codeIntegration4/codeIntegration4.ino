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
#include <TinyGPS++.h>

/********************************************************Variable********************************************/
float referenceAltitude,actualAltitude,realAltitude,tiltx,tilty,orientation,i,t;
double temperature,gAltitude,pressure,latitude_val, longitude_val;
int period=1000, flag,s , sats,j, Actualtime,UTCtime,UTCtime1,UTCtime2;
uint8_t hr, mins, sec;
unsigned long time_now=0;
char storedArray[30], buffers[100], arrays[j];
const int chipSelect = BUILTIN_SDCARD;
uint32_t timer = millis();
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
TinyGPSPlus gps;

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
void Program();//Function for saving the program in SDcard and reading the saved program in SDcard

void rtc_set();
void rtc_loop();

void VD();
void buffer_fun();

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

Serial.println("Position you MPU9250 flat and don’t move it – Calibrating...");
delay(1000);
myMPU9250.autoOffsets();
Serial.println("Done!");
myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
myMPU9250.enableAccDLPF(true);
myMPU9250.setAccDLPF(MPU9250_DLPF_6);
}

void mpu_tilt() {
xyzFloat gValue = myMPU9250.getGValues();
xyzFloat angle = myMPU9250.getAngles();
tiltx =angle.x;
tilty = angle.y;
//orientation = myMPU9250.getOrientationAsString(); 
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
  hr = gps.time.hour(); 
  mins= gps.time.minute();  
  sec= gps.time.second();  
        
 latitude_val= (GPS.latitude, 4); 
 char lat_dir= (GPS.lat);
 longitude_val= (GPS.longitude, 4); 
 char lng_dirn=(GPS.lon);
 gAltitude = GPS.altitude;
 sats = (int)GPS.satellites;
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
  sprintf(storedArray,"1033, %d:%d:%d, %d, %d, %d, %d, %d:%d:%d, %d, %d%d, %d%d, %d, %d,%d ",UTCtime1, UTCtime2, s, realAltitude,temperature, volts, pressure,hr, mins, sec, gAltitude, latitude_val, lat_direction, longitude_val, long_direction, sats, tiltx, tilty);   
  for (j = 0; j < 5; j++){
        dataFile.print(storedArray);
        dataFile.println();
  }
  dataFile.close();
  //Load program
  dataFile = SD.open("sd_card.csv");
  dataFile.read(&arrays[j],arrays[j]);
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
sprintf(buffers,"1033, %d:%d:%d, %f, %lf, %d, %lf, %02d:%02d:%02d, %lf, %lf%c, %lf%c, %d, %f,%f ",UTCtime1, UTCtime2, s, realAltitude,temperature, volts, pressure,hr, mins, sec, 
gAltitude, latitude_val, lat_dir, longitude_val, lng_dir, sats, tiltx, tilty); 
Serial.print(buffers);
}