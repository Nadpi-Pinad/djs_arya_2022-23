/********************************LIBRARY FUNCTIONS**************************/
#include <Wire.h>
#include <Adafruit_BMP280.h>     //BMP library (verify once)
#include <MPU9250_WE.h>          //MPU library
#include <EEPROM.h>
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

#define launch_h 25
#define rocket_sep_h1 600
//#define rocket_sep_h2 725
#define cansat_release_h 550
#define probe_release_h 500
#define probe_parachute_h 200
#define probe_upright_h 100
#define ground_landed_h 50

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

  //CODE WHICH SENDS TELEMETRY AT 1 HZ
  if(millis() > previousTime + 1000){
    probe_buffer_function();
    telemetry_function();
    previousTime = millis();
  }

  //CODE WHICH CHECKS FOR GS COMMANDS CONSTANTLY AND ALTITUDE VERIFICATION FOR SS
    softwareStateLoop();
    gui_commands_function();
    bmp_loop_function();
}
