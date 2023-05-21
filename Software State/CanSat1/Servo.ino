
/********************************SERVO SETUP AND VARIOUS TRIGGER MECHANISMS**********************************/
void servo_setup_function(){
    heatShieldServo.attach(33);
    parachuteServo.attach(10);
    flagServo.attach(36);
}

void heat_shield_release_function(){
  heatShieldServo.write(180);
  heatShieldTime = millis();
  heatShieldCount=EEPROM.read(heatShieldTimeLocation);
  
  while(millis() - heatShieldTime <= 5000){
    if(heatShieldCount>5)     
    {
      break;
    }
    if(millis() > previousTime + 1000)
    {
      heatShieldCount++;
      EEPROM.write(heatShieldTimeLocation,heatShieldCount);
      probe_buffer_function();
      telemetry_function();
      gui_commands_function();
      previousTime = millis();
    }
  }
  
  heatShieldServo.write(90);
}  
void probe_upright_function(){
  heatShieldServo.write(180);
  heatShieldTime = millis();

  while(millis() - heatShieldTime <= 6000){
    if(millis() > previousTime + 1000){
    probe_buffer_function();
    telemetry_function();
    gui_commands_function();
    previousTime = millis();
  }}
  
  heatShieldServo.write(90);
}

void probe_parachute_function(){
  for(servoAngle = 0; servoAngle < 90; servoAngle++){                                  
    parachuteServo.write(servoAngle);               
    delay(15);                   
  } 
}

void flag_release_function(){
  mastRaised = EEPROM.read(msLocation);
     if(mastRaised=='N')
        {
          flagServo.write(0);
         }
     else if(mastRaised=='M')
        {
          flagServo.write(180);
        }
  for(servoAngle = 0; servoAngle < 180; servoAngle++){                                  
    flagServo.write(servoAngle);               
    delay(15);                   
  }
}
