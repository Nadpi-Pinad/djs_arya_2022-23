char userInput;
unsigned long delayTime;

void toggleLed() ;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0){
    userInput = Serial.read();
    Serial.print("The user input is ");
    Serial.println(userInput);
  }

  if(userInput == 'a'|| userInput == 'A')
  digitalWrite(13,HIGH);

  else if(userInput == 'b'|| userInput == 'B')
  digitalWrite(13,LOW);

  else if(userInput == 'c'|| userInput == 'C'){
    
    Serial.println("Enter delay amount: ");
    while(Serial.available()==0){}
    delayTime=Serial.parseInt();
    
    if(delayTime>0 && (userInput == 'c' || userInput == 'C')){
    toggleLed();
    }
  }

  else if(userInput == 'd'||userInput == 'D'){
    digitalWrite(13,LOW);
    Serial.end();
  }
  

}

void toggleLed(){
      digitalWrite(13,HIGH);
      delay(delayTime);
      digitalWrite(13,LOW);
      delay(delayTime);

      if(Serial.available()==0)
      toggleLed();
}
