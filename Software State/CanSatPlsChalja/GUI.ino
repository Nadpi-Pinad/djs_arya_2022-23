/********************************GUI COMMANDS FUNCTION()**********************************/
//CHANGE TO SERIAL 5 RX2-20 TX2-21    
void gui_commands_function(){
  if(Serial5.available()>0){
    guiCommand = Serial5.read();
    switch(guiCommand){
      
      case '1':
      probeOnOff = 1;
      EEPROM.write(probeOnOffLocation,probeOnOff);
      cmdEcho = "CXON";
      guiCommand = 0;
      break;

      case '2':
      probeOnOff = 0;
      EEPROM.write(probeOnOffLocation,probeOnOff);
      cmdEcho = "CXOFF";
      guiCommand = 0;
      break;

      case '3':
      heatShieldServo.write(0);
      guiCommand = 0;
      break;

      case '4':
      heatShieldServo.write(90);
      guiCommand = 0;
      break;

      case '5':
      flagServo.write(0);
      guiCommand = 0;
      break;
      
      case '6':
      flagServo.write(180);
      guiCommand = 0;
      break;

      case '7':
      heatShieldServo.write(180);
      guiCommand = 0;
      break;

      case '8':
      parachuteServo.write(0);
      guiCommand = 0;
      break;
      
      case '9':
      parachuteServo.write(90);
      guiCommand = 0;
      break;
      
      case 'a':
      parachuteServo.write(180);
      guiCommand = 0;
      break;

      case 'c':
      digitalWrite(trig, LOW);   
      digitalWrite(led,LOW);
      delay(750);               
      digitalWrite(trig, HIGH);   
      digitalWrite(led,HIGH); 
      guiCommand = 0;
      break;

      case 'b':
      referenceAltitude = abs(bmp.readAltitude());
      EEPROM.put(referenceAltitudeLocation,referenceAltitude);
      Serial5.println("Altitude reference set by command:");
      Serial5.println(referenceAltitude);
      cmdEcho = "CAL";
      guiCommand = 0;
      break;

      case 'p': 
      packetCount = 1;
      EEPROM.put(packetCountLocation,0);                     //PACKET COUNT IS STORED AT LOCATION 50
      guiCommand = 0;
      break;

      case 's':
      softwareState = 0;
      Serial5.println("Software state has been reset");      
      guiCommand = 0;
      break;

      case 'h':
      heat_shield_release_function();
      guiCommand = 0;
      break;

      case 'e':
      heatShieldCount=0;
      EEPROM.write(heatShieldTimeLocation,heatShieldCount);
      guiCommand=0;
      break;
      //MULTIPLE CASES FOR EEPROM BMP AND OTHER GUI COMMANDS

      case 'd':
      hsDeployed = 'N';
      parachuteDeployed = 'N';
      mastRaised = 'N';

      
//      EEPROM.put(heatShieldLocation,hsDeployed);
//      EEPROM.put(parachuteLocation,parachuteDeployed);
//      EEPROM.put(flagLocation,mastRaised);
      guiCommand=0;
      break;

      case 'i':
      heat_shield_release_function();
      Serial5.println("Heat Shield Release initiated");
      guiCommand = 0;
      break;

      case 'j':
      probe_parachute_function();
      Serial5.println("Parachute Release initiated");
      guiCommand = 0;
      break;

      case 'k':
      probe_upright_function();
      Serial5.println("Probe upright initiated");
      guiCommand = 0;
      break;

      case 'l':
      flag_release_function();
      Serial5.println("Flag Release has been initiated");
      guiCommand = 0;
      break;

      case 'r':
      activate_audio_beacon();
      guiCommand = 0;
      break;

      case 'x':
      digitalWrite(buzzerPin,HIGH);
      guiCommand = 0;
      break;
    }
  }
}
