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
   
  EEPROM.get(referenceAltitudeLocation,referenceAltitude);
}

void bmp_loop_function(){
    bmpAltitude = abs(bmp.readAltitude());
    altitude = abs(bmpAltitude - referenceAltitude);
    temperature = bmp.readTemperature();
    pressure = (bmp.readPressure()/1000);
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
