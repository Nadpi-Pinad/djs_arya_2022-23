/********************************VOLTAGE DIVIDER**********************************/
void voltagedivider_loop_function(){
  //Code will be changed according to requirements
  volts = dummyVolts[indexVolt];
  indexVolt++;
  if(indexVolt == 400){
    indexVolt = 0;
  }
}
