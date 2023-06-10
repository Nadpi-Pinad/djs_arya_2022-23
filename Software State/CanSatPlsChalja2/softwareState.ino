/********************************SOFTWARE STATE SETUP USEFUL FOR RESET CONDITION**********************************/
void softwareState_setup(){
  EEPROM.get(softwareStateLocation,softwareState);
}


void softwareStateLoop(){  
  if(softwareState==0){
    softwareState=1;
    EEPROM.write(softwareStateLocation,softwareState);
  }

  else if(softwareState==1){
    if(abs(altitude)>launch_h){
      softwareState=2;
      EEPROM.write(softwareStateLocation,softwareState);
  }}

  else if(softwareState==2){
    if(abs(altitude)>rocket_sep_h1){
      softwareState=3;
      EEPROM.write(softwareStateLocation,softwareState);
  }}

  else if(softwareState==3){
    if(abs(altitude)<cansat_release_h){
      softwareState = 4;
      EEPROM.write(softwareStateLocation,softwareState);
  }}

  else if(softwareState==4){
    if(abs(altitude)<probe_release_h){
      softwareState=5;
      EEPROM.write(softwareStateLocation,softwareState);
      probe_parachute_function();
      start_stop_camera_function();
      probe_parachute_function();
      if(hsDeployed == 'N'){
        heat_shield_release_function();
        hsDeployed = 'P';
      }
    }
  }

  else if(softwareState == 5){
    hsDeployed = 'P';
    if(abs(altitude)<probe_parachute_h){
      if(parachuteDeployed == 'N'){
        probe_parachute_function();
        parachuteDeployed = 'C';
      }
      softwareState = 6;
      EEPROM.write(softwareStateLocation,softwareState);
    }
  }

  else if(softwareState == 6){
    hsDeployed = 'P';
    parachuteDeployed = 'C';
    if(abs(altitude)<probe_upright_h){
      probe_upright_function();
      softwareState = 7;
      EEPROM.write(softwareStateLocation,softwareState);
    }
  }

  else if(softwareState == 7){
    hsDeployed = 'P';
    parachuteDeployed = 'C';
    if(abs(altitude)<ground_landed_h){
      softwareState = 8;
      EEPROM.write(softwareStateLocation,softwareState);
      start_stop_camera_function();
    }
  }

  else if(softwareState == 8){
    hsDeployed = 'P';
    parachuteDeployed = 'C';
     if(mastRaised == 'N'){
       flag_release_function();
       mastRaised = 'M';
    }
    activate_audio_beacon();
    softwareState = 9;
    EEPROM.write(softwareStateLocation,softwareState);
    probeOnOff = 0;
    EEPROM.write(probeOnOffLocation,probeOnOff);
  }
}
