/********************************VOLTAGE DIVIDER**********************************/
void voltagedivider_loop_function(){
  //Code will be changed according to requirements
  val = analogRead(batteryPin);
  volts = (val/resistorFactor)*referenceVolts;
}
