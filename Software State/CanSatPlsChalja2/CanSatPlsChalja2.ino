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
char c;


//VOLTAGE DIVIDER VARIABLES
double volts;
double dummyVolts[] = {7.1, 7.15, 7.33, 7.08, 7.18, 7.13, 7.2, 7.18, 7.13, 7.21, 7.2, 7.11, 7.21, 7.38, 7.17, 7.18, 7.18, 7.16, 7.31, 7.09, 7.12, 7.32, 7.21, 7.39, 7.25, 7.26, 7.25, 7.29, 7.14, 7.33, 7.21, 7.15, 7.08, 7.2, 7.23, 7.29, 7.37, 7.31, 7.14, 7.07, 7.38, 7.33, 7.13, 7.12, 7.34, 7.1, 7.16, 7.31, 7.37, 7.37, 7.01, 7.19, 7.33, 7.1, 7.2, 7.17, 7.35, 7.16, 7.19, 7.2, 7.16, 7.16, 7.08, 7.38, 7.02, 7.26, 7.28, 7.08, 7.1, 7.32, 7.33, 7.21, 7.34, 7.15, 7.39, 7.32, 7.36, 7.09, 7.21, 7.08, 7.39, 7.01, 7.34, 7.04, 7.29, 7.23, 7.12, 7.1, 7.32, 7.24, 7.16, 7.07, 7.23, 7.07, 7.34, 7.21, 7.08, 7.26, 7.1, 7.36, 7.12, 7.09, 7.15, 7.39, 7.19, 7.1, 7.16, 7.14, 7.24, 7.03, 7.25, 7.1, 7.01, 7.37, 7.23, 7.01, 7.32, 7.2, 7.34, 7.15, 7.25, 7.22, 7.36, 7.13, 7.4, 7.03, 7.11, 7.14, 7.29, 7.04, 7.29, 7.26, 7.29, 7.38, 7.28, 7.39, 7.07, 7.08, 7.13, 7.05, 7.06, 7.13, 7.33, 7.17, 7.1, 7.11, 7.33, 7.16, 7.27, 7.13, 7.16, 7.12, 7.06, 7.06, 7.34, 7.14, 7.04, 7.35, 7.39, 7.31, 7.13, 7.31, 7.06, 7.11, 7.2, 7.27, 7.18, 7.16, 7.02, 7.39, 7.26, 7.15, 7.24, 7.3, 7.15, 7.17, 7.27, 7.33, 7.27, 7.28, 7.13, 7.22, 7.18, 7.16, 7.04, 7.13, 7.22, 7.11, 7.09, 7.29, 7.38, 7.08, 7.33, 7.0, 7.22, 7.01, 7.14, 7.28, 7.1, 7.31, 7.07, 7.27, 7.16, 7.35, 7.08, 7.18, 7.13, 7.11, 7.25, 7.18, 7.08, 7.01, 7.33, 7.22, 7.21, 7.32, 7.25, 7.15, 7.38, 7.33, 7.4, 7.37, 7.24, 7.3, 7.14, 7.03, 7.26, 7.01, 7.08, 7.27, 7.02, 7.16, 7.13, 7.39, 7.02, 7.29, 7.28, 7.39, 7.13, 7.33, 7.36, 7.39, 7.38, 7.26, 7.02, 7.05, 7.24, 7.33, 7.13, 7.07, 7.3, 7.24, 7.1, 7.31, 7.06, 7.22, 7.02, 7.23, 7.38, 7.31, 7.37, 7.32, 7.25, 7.4, 7.22, 7.28, 7.18, 7.09, 7.11, 7.13, 7.22, 7.35, 7.1, 7.24, 7.21, 7.15, 7.13, 7.35, 7.4, 7.15, 7.16, 7.02, 7.19, 7.25, 7.2, 7.17, 7.04, 7.07, 7.03, 7.3, 7.2, 7.1, 7.27, 7.27, 7.39, 7.4, 7.35, 7.28, 7.09, 7.03, 7.27, 7.27, 7.4, 7.06, 7.15, 7.09, 7.27, 7.07, 7.28, 7.3, 7.36, 7.02, 7.31, 7.34, 7.06, 7.01, 7.29, 7.08, 7.03, 7.3, 7.34, 7.29, 7.22, 7.11, 7.35, 7.19, 7.22, 7.24, 7.28, 7.16, 7.27, 7.25, 7.35, 7.07, 7.12, 7.29, 7.31, 7.13, 7.1, 7.38, 7.33, 7.1, 7.22, 7.29, 7.17, 7.09, 7.36, 7.18, 7.11, 7.17, 7.32, 7.32, 7.21, 7.2, 7.3, 7.03, 7.22, 7.04, 7.0, 7.11, 7.35, 7.2, 7.16, 7.39, 7.25, 7.4, 7.26, 7.08, 7.33, 7.23, 7.04, 7.15, 7.12, 7.13, 7.22, 7.39, 7.33, 7.22, 7.31, 7.05, 7.18, 7.08, 7.03, 7.03, 7.15, 7.23, 7.33, 7.16, 7.15, 7.08, 7.22, 7.37, 7.01, 7.05, 7.31, 7.29, 7.19, 7.05, 7.34, 7.08};
int indexVolt;

//LED CAMERA AND BUZZER
int led = 13;                    //TEENSY BUILT-IN LED
int trig = 3;                    //Camera Trigger 
int buzzerPin = 37;              //BUZZER PIN

char probeBuffer[100];            //Will store telemetry data ** CHANGE THE SIZE!!!
bool probeOnOff;                 //This decides whether to send telemetry or not

//SERVO ROTATION
unsigned long previousTime = 0;  //Used for delay generation
unsigned long heatShieldTime = 0;//Used for calculating the rotations of heat shield servo
unsigned long heatShieldReset;
char guiCommand;                 //Stores recent command sent by GS

char hsDeployed = 'N';
char parachuteDeployed = 'N';
char mastRaised = 'N';
int servoAngle;
int heatShieldCount=0; 

String cmdEcho = "CXON";
char mode = 'F';

const int chipSelect = BUILTIN_SDCARD; //SD_CARD

/********************************DEFINITIONS**************************/
#define GPSSerial Serial1              //GPS COMMUNICATES VIA SERIAL 1
#define GPSECHO false 

//EEPROM LOCATIONS ARE MENTIONED HERE

#define referenceAltitudeLocation 400
#define packetCountLocation 50
#define softwareStateLocation 500
#define probeOnOffLocation 25
#define heatShieldTimeLocation 300

#define MPU9250_ADDR 0x68        //MPU I2C ADDRESS

#define launch_h 25
#define rocket_sep_h1 650
//#define rocket_sep_h2 725
#define cansat_release_h 600
#define probe_release_h 500
#define probe_parachute_h 200
#define probe_upright_h 150
#define ground_landed_h 15

 
/********************************OBJECT DECLARATION**************************/
Adafruit_BMP280 bmp;
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
time_t RTCTime;
Adafruit_GPS GPS(&GPSSerial);
Servo heatShieldServo;
Servo parachuteServo;
Servo flagServo;
File myFile; //SDCARD

void probe_buffer_function();
void telemetry_function();
void voltagedivider_loop_function();
void softwareState_setup();
void softwareStateLoop();
void bmp_setup_function();
void mpu_setup_function();
void rtc_setup_function();
void gps_setup_function();
void servo_setup_function();
void camera_setup_function();
void gui_commands_function();
void bmp_loop_function();
void probe_buffer_function();
void telemetry_function();

/********************************SETUP**************************/
void setup() {
  // put your setup code here, to run once:
  Serial5.begin(9600);
  Serial.begin(9600);
  pinMode(buzzerPin,OUTPUT);  
  digitalWrite(buzzerPin,LOW);
  
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

  //CODE WHICH CHECKS FOR GS COMMANDS CONSTANTLY AND ALTITUDE VERIFICATION FOR SS
    softwareStateLoop();
    gui_commands_function();
    bmp_loop_function();

  //CODE WHICH SENDS TELEMETRY AT 1 HZ
  if(millis() > previousTime + 1000){
//    probe_buffer_function();
//    telemetry_function();
    gps_loop_function();
    previousTime = millis();
  }
}
