/**************Tasks to be done***************/
//Storing values from BMP280 in EEPROM

/**************Library***************/
#include <EEPROM.h> //EEPROM library
#include <Adafruit_BMP280.h> //adafruit BMP280 library
#include <Servo.h>

/**************Variable***************/
double referenceAltitude;        //Ground station altitude
double bmpAltitude,altitude;     //Height given by BMP & telemetry height
byte referenceAltitudeFlag;      //Reference altitude flag
bool descent_done;
int i=0;
char guiCommand;

/**************Define***************/
#define payload_release_h 500
#define altitudeFlag 200 //EEPROM
#define referenceAltitudeLocation 400 //EEPROM
#define descent_done_location 300 //EEPROM
#define rocket_sep_h1 625
#define rocket_sep_h2 750
 
/**************UserDefine Function declaration***************/
void bmp_setup_function();
void bmp_loop_function();

/**************object declaration***************/
Servo probe_release_servo;
Adafruit_BMP280 bmp; // I2C Interface

/**************Setup***************/
void setup() {
Serial.begin(9600);  
bmp_setup_function();

probe_release_servo.attach(10);
probe_release_servo.write(0); //subject to change

descent_done = EEPROM.read(descent_done_location);
descent_done=false;

EEPROM.get(referenceAltitudeLocation,referenceAltitude);
}

/**************Loop***************/
void loop() 
{
  commands();
  bmp_loop_function();
  if (abs(altitude) >= rocket_sep_h1 && abs(altitude) <= rocket_sep_h2)
  {
    descent_done=true;
    Serial.print("Ascent_done");
    EEPROM.write(descent_done_location, descent_done);
   }
    if (abs(altitude)<=payload_release_h)
   {
     if(descent_done==true)
     {
      Serial.print("Servo_done");
      probe_release_servo.write(180);
    }
   }}
   
/****************************************************BMP SETUP FUNCTION**************************************************/
 void bmp_setup_function() {
  //Initialize BMP
  if (!bmp.begin(0x76)) {
  Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void bmp_loop_function()
{
  bmpAltitude = abs(bmp.readAltitude());
  altitude = abs(bmpAltitude - referenceAltitude);
  i+=1;
  Serial.println(altitude);
}

void commands()
{
  if(Serial.available()>0){
    guiCommand = Serial.read();
    switch(guiCommand){
      case '1':
      referenceAltitude = abs(bmp.readAltitude());
      EEPROM.put(referenceAltitudeLocation,referenceAltitude);
      guiCommand=0;
      break;

      case '2':
      EEPROM.put(referenceAltitudeLocation,0);
      guiCommand=0;
      break;
    }}}
