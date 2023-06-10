/********************************GPS SETUP AND LOOP FUNCTIONS**********************************/
void gps_setup_function() {

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
}

void gps_loop_function() {
  c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived())
  {
    Serial.print(GPS.lastNMEA());
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
        gpsSatellites = (int)GPS.satellites;}
//  if (GPS.fix) {
//    sprintf(gpsArray, "%d:%d:%d,%.4lf,%.4lf,%.1lf,%d",
//            GPS.hour, GPS.minute, GPS.seconds,
//            GPS.latitude, GPS.longitude,
//            GPS.altitude, GPS.satellites);
//    Serial5.println(gpsArray);
    probe_buffer_function();
    telemetry_function();
  }

  
