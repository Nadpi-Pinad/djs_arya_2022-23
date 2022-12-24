char storedArray[30];

#include <SD.h>
#include <SPI.h>
File dataFile;
const int chipSelect = BUILTIN_SDCARD;
void setup() {

if (!SD.begin(chipSelect)) {
Serial.println("initialization failed!");
return;
}
Serial.println("initialization done.");
}

void loop() {
saveProgram();
loadProgram();
}

void saveProgram(){
  dataFile = SD.open("sdcard_t.csv", FILE_WRITE);
  int i;
  /*String str = "My name is Janhvi"; 
  int str_len = str.length() + 1; 
  char char_array[str_len];
  str.toCharArray(char_array[i], str_len);*/
//  char array[][20] = { "My", "name", "is","Janhvi" };
//  for (i = 0; i < 4; i++){
//        dataFile.print(array[][i]);
//  }
  sprintf(storedArray,"My,name,is,Janhvi,");
  for (i = 0; i < 5; i++){
        dataFile.print(storedArray);
        dataFile.println();
  }
  //dataFile.println();
  dataFile.close();
}

// load program from card
void loadProgram(){
 int i;
 char array[i];
 
  dataFile = SD.open("sd_card.csv");
  dataFile.read(&array[i],array[i]);
  dataFile.close(); 
}
