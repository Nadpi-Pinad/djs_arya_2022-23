

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
