/*
 * TimeRTC.pde
 * example code illustrating Time library with Real Time Clock.
 * 
 */

#include <TimeLib.h>

int Actualtime,UTCtime,UTCtime1,UTCtime2 ;

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void digitalClockDisplay() {
  // digital clock display of the time
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
void setup()  {
  // set the Time library to use Teensy 4.0's RTC to keep time
  setSyncProvider(getTeensy3Time);

  Serial.begin(9600);
  while (!Serial);  // Wait for Arduino Serial Monitor to open
  delay(100);
  if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("According to UTC Time");
  }
}

void loop() {
  if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }
  digitalClockDisplay();  
  delay(1000);
}
