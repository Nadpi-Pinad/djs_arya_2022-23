/**************Tasks to be done***************/
// Capture the photo/video according to the user input

/**************Library***************/

/**************Variable***************/
int trig = 3;
char temp;

/**************Define***************/

/**************UserDefine Function declaration***************/
void video();

/**************Setup***************/
void setup() 
{
  pinMode(trig, OUTPUT);         
  Serial.begin(9600); 
  digitalWrite(trig, HIGH); 
}

/**************Loop***************/
// Hold HIGH and trigger quick (<250ms) LOW to take a photo. Holding LOW and trigger HIGH starts/stops video recording

void loop() 
{
  if(Serial.available()>0)
  {
    temp=Serial.read();
    Serial.print(temp);
    video();
  }
}

/**************Function Description***************/
void video()
{
  if (temp=='1')
  {
    digitalWrite(trig, LOW);
    delay(500);
    digitalWrite(trig, HIGH);
    delay(100);      
  }
  else if(temp=='2') 
  {
    digitalWrite(trig, LOW);
    delay(500);
    digitalWrite(trig, HIGH);
  }
}
