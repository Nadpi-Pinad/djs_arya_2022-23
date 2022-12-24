#include <Adafruit_GPS.h>
#define GPSSerial Serial
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false
uint32_t timer = millis();
void GPS_setup();
void GPS_setup()
{
 GPS.begin(9600);
 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
 GPSSerial.println(PMTK_Q_RELEASE);
}
void setup()
{
 Serial.begin(115200);
 Serial.println("Adafruit GPS library basic parsing test!");
 GPS.begin(9600);
 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
 delay(1000);
 GPSSerial.println(PMTK_Q_RELEASE); 
}
void loop ()
{
 char c = GPS.read();
 if (GPSECHO)
 if (c) Serial.print(c);
 if (GPS.newNMEAreceived()) {
 Serial.print(GPS.lastNMEA()); 
 if (!GPS.parse(GPS.lastNMEA())) 
 return; 
 }
 if (millis() - timer > 2000) {
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
