/**************Tasks to be done***************/
//Storing values in SDcard using Teensy4.1

/**************Library***************/
#include <SD.h> //SD card library
#include <SPI.h> //SPI interface

/**************Variable***************/
char storedArray[30];
File dataFile;
const int chipSelect = BUILTIN_SDCARD;
int i;
char array[i];
 
/**************Define***************/

/**************UserDefine Function declaration***************/
void saveProgram(); //Function for saving the program in SDcard 
void loadProgram(); //Function for reading the saved program in SDcard
/**************Setup***************/
void setup() {

if (!SD.begin(chipSelect)) {
Serial.println("initialization failed!");
return;
}
Serial.println("initialization done.");
}

/**************Loop***************/
void loop() {
saveProgram();
loadProgram();
}

/**************Function Description***************/
void saveProgram(){
  dataFile = SD.open("sdcard_t.csv", FILE_WRITE);
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
  dataFile = SD.open("sd_card.csv");
  dataFile.read(&array[i],array[i]);
  dataFile.close(); 
}








