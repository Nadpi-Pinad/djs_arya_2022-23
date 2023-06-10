/********************************PROBE BUFFER CREATION FUNCTION()**********************************/

void sdcard();
void telemetry_function();

void probe_buffer_function(){
  if(probeOnOff == 1){
   bmp_loop_function();
   rtc_loop_function();
   mpu_loop_function();
   gps_loop_function();
   voltagedivider_loop_function();
   sdcard();
   

   sprintf(probeBuffer,"1033,%d:%d:%d,%d,%c,%d,%.1lf,%c,%c,%c,%.1lf,%.1lf,%.1lf,%d:%d:%d,%.1lf,%.4lf,%.4lf,%d,%.2lf,%.2lf,",
           h,m,s,packetCount,mode,softwareState,altitude,hsDeployed,parachuteDeployed,mastRaised,
           temperature,volts,pressure,gpsHour,gpsMinute,gpsSecond,gpsAltitude,gpsLatitude1,gpsLongitude1,
           gpsSatellites,tiltX,tiltY);
   strcat(probeBuffer,cmdEcho.c_str());
   packetCount=packetCount+1;
   EEPROM.put(packetCountLocation,packetCount);
  }}

/********************************SENDING PROBE TELEMETRY FUNCTION()**********************************/
void telemetry_function(){
  if(probeOnOff == 1){
    for(int i = 0;i<100;i++){
      Serial5.print(probeBuffer[i]);
    }
    Serial5.println();
  }
} 
