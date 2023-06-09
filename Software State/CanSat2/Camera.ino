/********************************CAMERA SETUP AND LOOP FUNCTION**********************************/
void camera_setup_function(){
  pinMode(led,OUTPUT);
  pinMode(trig,OUTPUT);

  digitalWrite(led,HIGH);
  digitalWrite(trig,HIGH);
}

void start_stop_camera_function(){
  digitalWrite(trig, LOW);   
  digitalWrite(led,LOW);
  delay(750);                      //A low to high pulse greater than 500 ms for video
  digitalWrite(trig, HIGH);   
  digitalWrite(led,HIGH);
}
