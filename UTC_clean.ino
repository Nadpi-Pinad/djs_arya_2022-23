/**Tasks to be done***/
//Measuring UTC time using Teensy 4.1

/**Library***/
#include <TimeLib.h> //UTC time library 

/**Variable***/
int Actualtime,UTCtime,UTCtime1,UTCtime2;

/**Define***/
#define TIME_HEADER  "T"   // Header tag for serial time sync message

/**UserDefine Function declaration***/
void rtc_printDigits(int digits);  // utility function for digital clock display
void rtc_digitalClockDisplay();   // digital clock display of the time
unsigned long processSyncMessage() ;

/**Setup***/
void setup(){
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

 /**Loop***/
 void loop() {
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
/**Function Description***/
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
