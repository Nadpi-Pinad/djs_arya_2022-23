#include <SoftwareSerial.h>
  
    SoftwareSerial xbee= SoftwareSerial(2, 3); // RX, TX

    
    int temp;
    
    void setup()
    {
     Serial.begin(9600); 
     pinMode(2,INPUT);
     pinMode(3,OUTPUT);
     Serial.println( "Arduino started receiving bytes via XBee" );
     
       // set the data rate for the SoftwareSerial port
       xbee.begin(9600);
    }
    
    void loop()  {
     if(xbee.available()>0){
     temp=xbee.read();}
  if (temp=='1'){
      Serial.println("Tulsi");
      xbee.println("Tulsi");
      delay(1000);
      }
 else if(temp=='2') {}

    }
//          xbee.println("Tulsi");}
//     Serial.println(temp); 
