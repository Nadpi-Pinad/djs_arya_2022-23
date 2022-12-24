int trig = 3;
char temp;
void video();
void setup() {                
 pinMode(trig, OUTPUT);         
Serial.begin(9600); 
digitalWrite(trig, HIGH); 
}

// Hold HIGH and trigger quick (<250ms) LOW to take a photo. Holding LOW and trigger HIGH starts/stops video recording

void loop() {
  if(Serial.available()>0){
    temp=Serial.read();
Serial.print(temp);
video();}}
void video(){
  if (temp=='1'){
      digitalWrite(trig, LOW);   
      delay(500);               
      digitalWrite(trig, HIGH);    
delay(100);      
      }
 else if(temp=='2') {
   digitalWrite(trig, LOW);   
      delay(500);               
      digitalWrite(trig, HIGH); }

    }
