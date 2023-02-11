#include <EEPROM.h>

int packetCount,flag;
void setup() {
 Serial.begin(9600);
 
 packetCount=EEPROM.read(5);
 flag=EEPROM.read(8);
 
 if(flag==0)
   {
      packetCount=0;
      EEPROM.write(5,packetCount);
      //EEPROM.write(5,packetCount++);
      
      flag=1;
      EEPROM.write(8,flag);
   }
 else
    {
      packetCount=EEPROM.read(5);
      //packetCount++;
      EEPROM.write(5,packetCount);
      
    }
 
}
void loop()
  {
    Serial.print("Packet count no =>");
    Serial.println(packetCount); 
    packetCount++;
    EEPROM.write(5,packetCount);
    delay(1000);
  }
