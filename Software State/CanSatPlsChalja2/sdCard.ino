/********************************SD_CARD_FUNCTION()**********************************/
void sdcard()
{
  myFile= SD.open("Flight_1033.csv", FILE_WRITE);
   if (myFile){
    Serial.println(probeBuffer);
    myFile.print(probeBuffer);
    myFile.println();
    myFile.close();
}}
