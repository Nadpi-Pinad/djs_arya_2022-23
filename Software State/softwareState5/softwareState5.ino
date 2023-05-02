/********************************LIBRARY FUNCTIONS**************************/
#include <Wire.h>
#include <Adafruit_BMP280.h>     //BMP library (verify once)
#include <MPU9250_WE.h>          //MPU library
#include <EEPROM.h>              //EEPROM library
#include <TimeLib.h>             //UTC Time library
#include <String.h>              //For displaying string
#include <Adafruit_GPS.h>        //GPS library
#include <Servo.h>               //Servo Library
#include <SD.h>

/********************************VARIABLE DECLARATION**************************/
byte softwareState;              //Stores software state
byte softwareStateFlag;          //FOR RESET CONDITION

double referenceAltitude;        //Ground station altitude
double bmpAltitude,altitude;     //Height given by BMP & telemetry height
byte referenceAltitudeFlag;      //Reference altitude flag
double temperature;              //BMP280 temperature

double tiltX,tiltY;              //MPU gives orientation angles
xyzFloat angle;                  //Angles in vector form

//EEPROM VARIABLES TO BE KEPT AFTER FINALISING
int packetCount;

int h,m,s;                       //Stores UTC hour,min,sec,hundredth of sec

//ALL THE GPS PARAMETERS ARE PRESENT
int gpsHour, gpsMinute, gpsSecond, gpsSatellites;
float gpsAltitude, gpsLatitude1, gpsLongitude1;


//VOLTAGE DIVIDER VARIABLES
const float referenceVolts = 3.3;
const float R1 = 4700;
const float R2 = 1000;
const float resistorFactor = 1023.0*(R2/(R1 + R2));
const int batteryPin = A4;
int val;
double volts;


int led = 13;                    //TEENSY BUILT-IN LED
int trig = 3;                    //Camera Trigger 
int buzzerPin = 20;              //BUZZER PIN

char probeBuffer[95];            //Will store telemetry data ** CHANGE THE SIZE!!!
bool probeOnOff = 0;             //This decides whether to send telemetry or not

unsigned long previousTime = 0;  //Used for delay generation
unsigned long heatShieldTime = 0;//Used for calculating the rotations of heat shield servo
char guiCommand;                 //Stores recent command sent by GS
unsigned int servoSpeed;         //Decides the speed of the servo motor (0-max CW,90-zero,180-max ACW)
char hsDeployed = 'N';
char parachuteDeployed = 'N';
char mastRaised = 'N';
int servoAngle;

char cmdEcho[10];
String cmdCommand;

const int chipSelect = BUILTIN_SDCARD; //SD_CARD

/********************************DEFINITIONS**************************/
#define GPSSerial Serial1        //GPS COMMUNICATES VIA SERIAL 1
#define GPSECHO false 

#define MPU9250_ADDR 0x68        //MPU I2C ADDRESS

#define launch_h 3
#define rocket_sep_h1 20
//#define rocket_sep_h2 725
#define cansat_release_h 16
#define probe_release_h 12
#define probe_parachute_h 8
#define probe_upright_h 4
#define ground_landed_h 2

/********************************OBJECT DECLARATION**************************/
Adafruit_BMP280 bmp;
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
time_t RTCTime;
Adafruit_GPS GPS(&GPSSerial);
Servo heatShieldServo;
Servo parachuteServo;
Servo flagServo;
File myFile; //SDCARD

void setup() {
  // put your setup code here, to run once:
  Serial5.begin(9600);
  
  packetCount = EEPROM.read(50);

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


void loop() {
  if(softwareState==0){
//    calibrate_all_sensors_function();
    softwareState=1;
    EEPROM.write(500,softwareState);
  }

  else if(softwareState==1){
    if(abs(altitude)>launch_h){
      softwareState=2;
      EEPROM.write(500,softwareState);
  }}

  else if(softwareState==2){
    if(abs(altitude)>rocket_sep_h1){
      softwareState=3;
      EEPROM.write(500,softwareState);
  }}

  else if(softwareState==3){
    if(abs(altitude)<cansat_release_h){
      softwareState = 4;
      EEPROM.write(500,softwareState);
  }}

  else if(softwareState==4){
    if(abs(altitude)<probe_release_h){
      softwareState=5;
      EEPROM.write(500,softwareState);
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
      EEPROM.write(500,softwareState);
    }
  }

  else if(softwareState == 6){
    if(abs(altitude)<probe_upright_h){
      probe_upright_function();
      softwareState = 7;
      EEPROM.write(500,softwareState);
    }
  }

  else if(softwareState == 7){
    if(abs(altitude)<ground_landed_h){
      softwareState = 8;
      EEPROM.write(500,softwareState);
      start_stop_camera_function();
    }
  }

  else if(softwareState == 8){
     if(mastRaised == 'N'){
       flag_release_function();
       mastRaised = 'M';
    }
    activate_audio_beacon();
    softwareState = 9;
    EEPROM.write(500,softwareState);
  }


  else if(softwareState == 9){
    probeOnOff = 0;
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
   
  referenceAltitudeFlag = EEPROM.read(200);           //FLAG AT LOCATION 200 AND ALTITUDE AT LOCATION 400
  
  if(referenceAltitudeFlag == 0){
   referenceAltitude = abs(bmp.readAltitude());
   EEPROM.put(400,referenceAltitude);
   referenceAltitudeFlag = 1;
   EEPROM.write(200,referenceAltitudeFlag); 
   Serial5.print("Setting reference altitude : ");
   Serial5.println(referenceAltitude);
  }

  else if(referenceAltitudeFlag == 1){
    EEPROM.get(400,referenceAltitude);
    Serial5.print("This is the reference altitude : ");
    Serial5.println(referenceAltitude);
  }  
}

void bmp_loop_function(){
    bmpAltitude = abs(bmp.readAltitude());
    altitude = abs(bmpAltitude - referenceAltitude);
    temperature = bmp.readTemperature();
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

  while(millis() - heatShieldTime <= 5000){
    if(millis() > previousTime + 1000){
    probe_buffer_function();
    telemetry_function();
    gui_commands_function();
    previousTime = millis();
  }}

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
  softwareStateFlag = EEPROM.read(100);           //FLAG AT LOCATION 100 AND STATE AT LOCATION 500

  
   if(softwareStateFlag == 0){
     softwareState = 0;
     softwareStateFlag = 1;
     EEPROM.write(100,softwareStateFlag);
    }

    else if(softwareStateFlag == 1){
      EEPROM.get(500,softwareState);
  } 
}

/********************************AUDIO BEACON ACTIVATION()**********************************/
void activate_audio_beacon(){
  digitalWrite(buzzerPin,HIGH);
}

/********************************SD_CARD_FUNCTION()**********************************/
void sdcard()
{
  myFile= SD.open("CodeData.csv", FILE_WRITE);
   if (myFile){
 Serial.println(buffers);
  myFile.print(buffers);
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
   sprintf(probeBuffer,"1033,%d:%d:%d,%d,F,%d,%.1lf,N,N,N,%.1lf,%.1lf,%d:%d:%d,%.1lf,%.4lf,%.4lf,%d,%.2lf,%.2lf,%s",
           h,m,s,packetCount,softwareState,altitude,temperature,volts,h,m,s,
           altitude,gpsLatitude1,gpsLongitude1,gpsSatellites,tiltX,tiltY,cmdEcho);
   packetCount=packetCount+1;
   EEPROM.put(50,packetCount);
  }
}

/********************************SENDING PROBE TELEMETRY FUNCTION()**********************************/
void telemetry_function(){
  if(probeOnOff == 1){
    for(int i = 0;i<95;i++){
      Serial5.print(probeBuffer[i]);
    }
    Serial5.println();
  }
}

/********************************GUI COMMANDS FUNCTION()**********************************/
//CHANGE TO SERIAL 2 RX2-7 TX2-8    
void gui_commands_function(){
  if(Serial5.available()>0){
    guiCommand = Serial5.read();
    switch(guiCommand){
      
      case '1':
      probeOnOff = 1;
      cmdCommand = "CXON";
      cmdCommand.toCharArray(cmdEcho,10);
      guiCommand = 0;
      break;

      case '2':
      probeOnOff = 0;
      cmdCommand = "CXOFF";
      cmdCommand.toCharArray(cmdEcho,10);
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
      EEPROM.write(200,referenceAltitudeFlag);
      EEPROM.put(400,referenceAltitude);
      Serial5.println("Altitude reference set");
      Serial5.println(referenceAltitude);
      guiCommand = 0;
      break;

      case 'p': 
      EEPROM.put(50,1);                     //PACKET COUNT IS STORED AT LOCATION 100
      guiCommand = 0;
      break;

      case 's':
      softwareState = 0;
      Serial5.println("Software state has been reset");
      guiCommand = 0;
      break;
     
      
      //MULTIPLE CASES FOR EEPROM BMP AND OTHER GUI COMMANDS
      
    }
  }
}
