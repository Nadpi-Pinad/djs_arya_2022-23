/********************************LIBRARY FUNCTIONS**************************/
#include <Wire.h>
#include <Adafruit_BMP280.h>     //BMP library (verify once)
#include <MPU9250_WE.h>          //MPU library
#include <EEPROM.h>              //EEPROM library
#include <TimeLib.h>             //UTC Time library
#include <String.h>              //For displaying string
#include <Adafruit_GPS.h>        //GPS library
#include <Servo.h>               //Servo Library
#include <SD.h>                  //SD CARD

/********************************VARIABLE DECLARATION**************************/
//SOFTWARE STATE
byte softwareState;              //Stores software state

//BMP280
double referenceAltitude;        //Ground station altitude
double bmpAltitude,altitude;     //Height given by BMP & telemetry height
byte referenceAltitudeFlag;      //Reference altitude flag
double temperature;              //BMP280 Temperature
double pressure;                 //BMP280 Pressure

//MPU9250
double tiltX,tiltY;              //MPU gives orientation angles
xyzFloat angle;                  //Angles in vector form

//EEPROM VARIABLES TO BE KEPT AFTER FINALISING
int packetCount;

//TEENSY RTC
int h,m,s;                       //Stores UTC hour,min,sec,hundredth of sec

//ALL THE GPS PARAMETERS ARE PRESENT
int gpsHour, gpsMinute, gpsSecond, gpsSatellites;
float gpsAltitude, gpsLatitude1, gpsLongitude1;


//VOLTAGE DIVIDER VARIABLES
const float referenceVolts = 3.3;
const float R1 = 4700;
const float R2 = 1000;
const float resistorFactor = 1023.0*(R2/(R1 + R2));
const int batteryPin = A15;
int val;
double volts;

//LED CAMERA AND BUZZER
int led = 13;                    //TEENSY BUILT-IN LED
int trig = 3;                    //Camera Trigger 
int buzzerPin = 20;              //BUZZER PIN

char probeBuffer[100];            //Will store telemetry data ** CHANGE THE SIZE!!!
bool probeOnOff;                 //This decides whether to send telemetry or not

//SERVO ROTATION
unsigned long previousTime = 0;  //Used for delay generation
unsigned long heatShieldTime = 0;//Used for calculating the rotations of heat shield servo
unsigned long heatShieldReset;
char guiCommand;                 //Stores recent command sent by GS
unsigned int servoSpeed;         //Decides the speed of the servo motor (0-max CW,90-zero,180-max ACW)
char hsDeployed = 'N';
char parachuteDeployed = 'N';
char mastRaised = 'N';
int servoAngle;
int heatShieldCount=0; 

String cmdEcho;
char mode = 'F';

const int chipSelect = BUILTIN_SDCARD; //SD_CARD

/********************************DEFINITIONS**************************/
#define GPSSerial Serial1              //GPS COMMUNICATES VIA SERIAL 1
#define GPSECHO false 

//EEPROM LOCATIONS ARE MENTIONED HERE
#define altitudeFlag 200
#define referenceAltitudeLocation 400
#define packetCountLocation 50
#define softwareStateLocation 500
#define softwareStateFlagLocation 100
#define probeOnOffLocation 25
#define heatShieldTimeLocation 300

#define MPU9250_ADDR 0x68        //MPU I2C ADDRESS

#define launch_h 3
#define rocket_sep_h1 20
//#define rocket_sep_h2 725
#define cansat_release_h 16
#define probe_release_h 12
#define probe_parachute_h 8
#define probe_upright_h 4
#define ground_landed_h 2

#define msLocation 500  
/********************************OBJECT DECLARATION**************************/
Adafruit_BMP280 bmp;
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
time_t RTCTime;
Adafruit_GPS GPS(&GPSSerial);
Servo heatShieldServo;
Servo parachuteServo;
Servo flagServo;
File myFile; //SDCARD

/********************************SETUP**************************/
void setup() {
  // put your setup code here, to run once:
  Serial5.begin(9600);
  
  EEPROM.get(packetCountLocation,packetCount);
  probeOnOff = EEPROM.read(probeOnOffLocation);

  bmp_setup_function();
  mpu_setup_function();
  rtc_setup_function();
  gps_setup_function();
  servo_setup_function();
  camera_setup_function();
  softwareState_setup();
  
  if (!SD.begin(chipSelect)) 
  {
    Serial5.println("Initialization failed!");
    return;
  }
  
}

/********************************LOOP**************************/
void loop() {
  if(softwareState==0){
//    calibrate_all_sensors_function();
    softwareState=1;
    EEPROM.write(softwareStateLocation,softwareState);
  }

  else if(softwareState==1){
    if(abs(altitude)>launch_h){
      softwareState=2;
      EEPROM.write(softwareStateLocation,softwareState);
  }}

  else if(softwareState==2){
    if(abs(altitude)>rocket_sep_h1){
      softwareState=3;
      EEPROM.write(softwareStateLocation,softwareState);
  }}

  else if(softwareState==3){
    if(abs(altitude)<cansat_release_h){
      softwareState = 4;
      EEPROM.write(softwareStateLocation,softwareState);
  }}

  else if(softwareState==4){
    if(abs(altitude)<probe_release_h){
      softwareState=5;
      EEPROM.write(softwareStateLocation,softwareState);
      start_stop_camera_function();
      if(hsDeployed == 'N'){
        heat_shield_release_function();
        hsDeployed = 'P';
      }
    }
  }

  else if(softwareState == 5){
    if(abs(altitude)<probe_parachute_h){
      if(parachuteDeployed == 'N'){
        probe_parachute_function();
        parachuteDeployed = 'C';
      }
      softwareState = 6;
      EEPROM.write(softwareStateLocation,softwareState);
    }
  }

  else if(softwareState == 6){
    if(abs(altitude)<probe_upright_h){
      probe_upright_function();
      softwareState = 7;
      EEPROM.write(softwareStateLocation,softwareState);
    }
  }

  else if(softwareState == 7){
    if(abs(altitude)<ground_landed_h){
      softwareState = 8;
      EEPROM.write(softwareStateLocation,softwareState);
      start_stop_camera_function();
    }
  }

  else if(softwareState == 8){
     if(mastRaised == 'N'){
       flag_release_function();
      EEPROM.write(msLocation,mastRaised);
       mastRaised = 'M';
      EEPROM.write(msLocation,mastRaised);
    }
    activate_audio_beacon();
    softwareState = 9;
    EEPROM.write(softwareStateLocation,softwareState);
  }


  else if(softwareState == 9){
    probeOnOff = 0;
    EEPROM.write(probeOnOffLocation,probeOnOff);
  }
  
  
  //CODE WHICH SENDS TELEMETRY AT 1 HZ
  if(millis() > previousTime + 1000){
    probe_buffer_function();
    telemetry_function();
    previousTime = millis();
  }

  //CODE WHICH CHECKS FOR GS COMMANDS CONSTANTLY AND ALTITUDE VERIFICATION FOR SS
    gui_commands_function();
    bmp_loop_function();
}


/********************************BMP SETUP AND LOOP FUNCTIONS**********************************/
void bmp_setup_function() {
  //Initialize BMP
  if (!bmp.begin(0x76)) {
  Serial5.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
   
  referenceAltitudeFlag = EEPROM.read(altitudeFlag);           //FLAG AT LOCATION 200 AND ALTITUDE AT LOCATION 400
  
  if(referenceAltitudeFlag == 0){
   referenceAltitude = abs(bmp.readAltitude());
   EEPROM.put(referenceAltitudeLocation,referenceAltitude);
   referenceAltitudeFlag = 1;
   EEPROM.write(altitudeFlag,referenceAltitudeFlag); 
   Serial5.print("Setting reference altitude : ");
   Serial5.println(referenceAltitude);
  }

  else if(referenceAltitudeFlag == 1){
    EEPROM.get(referenceAltitudeLocation,referenceAltitude);
  }  
}

void bmp_loop_function(){
    bmpAltitude = abs(bmp.readAltitude());
    altitude = abs(bmpAltitude - referenceAltitude);
    temperature = bmp.readTemperature();
    pressure = (bmp.readPressure()/1000);
}


/********************************MPU SETUP AND LOOP FUNCTIONS**********************************/
void mpu_setup_function(){
    Wire.begin();
    if(!myMPU9250.init()){}
    else{}
    delay(1000);
    myMPU9250.autoOffsets();
    myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
    myMPU9250.enableAccDLPF(true);
    myMPU9250.setAccDLPF(MPU9250_DLPF_6);
}

void mpu_loop_function(){
  xyzFloat angle = myMPU9250.getAngles();
  tiltX = angle.x;
  tiltY = angle.y;
}

/********************************RTC SETUP AND LOOP FUNCTION()**********************************/
void rtc_setup_function() {
    setSyncProvider(getTeensy3Time);
    if (timeStatus() != timeSet)
    {}
  }
  
  time_t getTeensy3Time() {
    return Teensy3Clock.get();
  }

void rtc_loop_function() {
    s = second();
    if (s >= 100)
    {
      s = 0;
    }
    if (hour() < 5 && minute() < 30) {
      h = (24 + hour() - 6);
      m = (60 + minute() - 30);
    }
    else if (hour() >= 5 && minute() >= 30) {
      h = hour() - 5;
      m = minute() - 30;
    }
    else if (hour() < 5 && minute() >= 30) {
      h = (24 + hour() - 5);
      m = minute() - 30;
    }
    else if (hour() >= 5 && minute() < 30) {
      h = hour() - 6;
      m = (60 + minute() - 30);
    }
  }

/********************************GPS SETUP AND LOOP FUNCTIONS**********************************/
void gps_setup_function(){
    Serial1.begin(9600);
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPSSerial.println(PMTK_Q_RELEASE);
    gpsLatitude1 = 1906.6795;
    gpsLongitude1 = 7250.8940;
    gpsAltitude = 22.4;
    gpsSatellites = 4;
}

void gps_loop_function(){
char c = GPS.read();
    if (GPSECHO)
      if (c) Serial.print(c);
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))
        return;
    }
    gpsHour = GPS.hour;
    gpsMinute = GPS.minute;
    gpsSecond = GPS.seconds;
    if (GPS.fix) {
      gpsLatitude1 = GPS.latitude;
      gpsLongitude1 = GPS.longitude;
      gpsAltitude = GPS.altitude;
      gpsSatellites = (int)GPS.satellites;
}}


/********************************CAMERA SETUP AND LOOP FUNCTION**********************************/
void camera_setup_function(){
  pinMode(led,OUTPUT);
  pinMode(trig,OUTPUT);

  digitalWrite(led,HIGH);
  digitalWrite(trig,HIGH);
}

void start_stop_camera_function(){
  digitalWrite(trig, LOW);   
  digitalWrite(led,LOW);
  delay(750);                      //A low to high pulse greater than 500 ms for video
  digitalWrite(trig, HIGH);   
  digitalWrite(led,HIGH);
}


/********************************SERVO SETUP AND VARIOUS TRIGGER MECHANISMS**********************************/
void servo_setup_function(){
    heatShieldServo.attach(33);
    parachuteServo.attach(10);
    flagServo.attach(36);
}

void heat_shield_release_function(){
  heatShieldServo.write(180);
  heatShieldTime = millis();
  heatShieldCount=EEPROM.read(heatShieldTimeLocation);
  
  while(millis() - heatShieldTime <= 5000){
    if(heatShieldCount>5)     
    {
      break;
    }
    if(millis() > previousTime + 1000)
    {
      heatShieldCount++;
      EEPROM.write(heatShieldTimeLocation,heatShieldCount);
      probe_buffer_function();
      telemetry_function();
      gui_commands_function();
      previousTime = millis();
    }
  }
  
  heatShieldServo.write(90);
}  
void probe_upright_function(){
  heatShieldServo.write(180);
  heatShieldTime = millis();

  while(millis() - heatShieldTime <= 6000){
    if(millis() > previousTime + 1000){
    probe_buffer_function();
    telemetry_function();
    gui_commands_function();
    previousTime = millis();
  }}
  
  heatShieldServo.write(90);
}

void probe_parachute_function(){
  for(servoAngle = 0; servoAngle < 90; servoAngle++){                                  
    parachuteServo.write(servoAngle);               
    delay(15);                   
  } 
}

void flag_release_function(){
  mastRaised = EEPROM.read(msLocation);
     if(mastRaised=='N')
        {
          flagServo.write(0);
         }
     else if(mastRaised=='M')
        {
          flagServo.write(180);
        }
  for(servoAngle = 0; servoAngle < 180; servoAngle++){                                  
    flagServo.write(servoAngle);               
    delay(15);                   
  }
}

/********************************VOLTAGE DIVIDER**********************************/
void voltagedivider_loop_function(){
  //Code will be changed according to requirements
  val = analogRead(batteryPin);
  volts = (val/resistorFactor)*referenceVolts;
}

/********************************SOFTWARE STATE SETUP USEFUL FOR RESET CONDITION**********************************/
void softwareState_setup(){
// softwareStateFlag = EEPROM.read(softwareStateFlagLocation);           //FLAG AT LOCATION 100 AND STATE AT LOCATION 500
//
//  
//   if(softwareStateFlag == 0){
//     softwareState = 0;
//     softwareStateFlag = 1;
//     EEPROM.write(softwareStateFlagLocation,softwareStateFlag);
//    }

   // else if(softwareStateFlag == 1){
     EEPROM.get(softwareStateLocation,softwareState);
  //} 
}

/********************************AUDIO BEACON ACTIVATION()**********************************/
void activate_audio_beacon(){
    digitalWrite(buzzerPin,LOW);
}

/********************************SD_CARD_FUNCTION()**********************************/
void sdcard()
{
  myFile= SD.open("CodeData.csv", FILE_WRITE);
   if (myFile){
    Serial.println(probeBuffer);
    myFile.print(probeBuffer);
    myFile.println();
    myFile.close();
}}

/********************************PROBE BUFFER CREATION FUNCTION()**********************************/
void probe_buffer_function(){
  if(probeOnOff == 1){
   bmp_loop_function();
   rtc_loop_function();
   mpu_loop_function();
   voltagedivider_loop_function();
   sdcard();

   //CMD ECHO,ALTITUDE,SS,PC,VOLTAGE
   // \n aage ki peeche ??
   sprintf(probeBuffer,"1033,%d:%d:%d,%d,%c,%d,%.1lf,%c,%c,%c,%.1lf,%.1lf,%.1lf,%d:%d:%d,%.1lf,%.4lf,%.4lf,%d,%.2lf,%.2lf,",
           h,m,s,packetCount,mode,softwareState,altitude,hsDeployed,parachuteDeployed,mastRaised,
           temperature,pressure,volts,h,m,s,altitude,gpsLatitude1,gpsLongitude1,
           gpsSatellites,tiltX,tiltY);
   strcat(probeBuffer,cmdEcho.c_str());
   packetCount=packetCount+1;
   EEPROM.put(packetCountLocation,packetCount);
  }
}

/********************************SENDING PROBE TELEMETRY FUNCTION()**********************************/
void telemetry_function(){
  if(probeOnOff == 1){
    for(int i = 0;i<100;i++){
      Serial5.print(probeBuffer[i]);
    }
    Serial5.println();
  }
}

/********************************GUI COMMANDS FUNCTION()**********************************/
//CHANGE TO SERIAL 5 RX2-20 TX2-21    
void gui_commands_function(){
  if(Serial5.available()>0){
    guiCommand = Serial5.read();
    switch(guiCommand){
      
      case '1':
      probeOnOff = 1;
      EEPROM.write(probeOnOffLocation,probeOnOff);
      cmdEcho = "CXON";
      guiCommand = 0;
      break;

      case '2':
      probeOnOff = 0;
      EEPROM.write(probeOnOffLocation,probeOnOff);
      cmdEcho = "CXOFF";
      guiCommand = 0;
      break;

      case '3':
      heatShieldServo.write(0);
      guiCommand = 0;
      break;

      case '4':
      heatShieldServo.write(90);
      guiCommand = 0;
      break;

      case '5':
      flagServo.write(0);
      guiCommand = 0;
      break;
      
      case '6':
      flagServo.write(180);
      guiCommand = 0;
      break;

      case '7':
      heatShieldServo.write(180);
      guiCommand = 0;
      break;

      case '8':
      parachuteServo.write(0);
      guiCommand = 0;
      break;
      
      case '9':
      parachuteServo.write(90);
      guiCommand = 0;
      break;
      
      case 'a':
      parachuteServo.write(180);
      guiCommand = 0;
      break;

      case 'c':
      digitalWrite(trig, LOW);   
      digitalWrite(led,LOW);
      delay(750);               
      digitalWrite(trig, HIGH);   
      digitalWrite(led,HIGH); 
      guiCommand = 0;
      break;

      case 'b':
      referenceAltitude = abs(bmp.readAltitude());
      referenceAltitudeFlag = 1;
      EEPROM.write(altitudeFlag,referenceAltitudeFlag);
      EEPROM.put(referenceAltitudeLocation,referenceAltitude);
      Serial5.println("Altitude reference set");
      Serial5.println(referenceAltitude);
      cmdEcho = "CAL";
      guiCommand = 0;
      break;

      case 'p': 
      packetCount = 1;
      EEPROM.put(packetCountLocation,0);                     //PACKET COUNT IS STORED AT LOCATION 50
      guiCommand = 0;
      break;

      case 's':
      softwareState = 0;
      Serial5.println("Software state has been reset");      
      guiCommand = 0;
      break;

      case 'h':
      heat_shield_release_function();
      guiCommand = 0;
      break;

      case 'e':
      heatShieldCount=0;
      EEPROM.write(heatShieldTimeLocation,heatShieldCount);
      guiCommand=0;
      break;
      //MULTIPLE CASES FOR EEPROM BMP AND OTHER GUI COMMANDS
      
    }
  }
}
