/*****Tasks to be done******/
//voltage divider using Teensy4.1

/*****Variable******/
const float referenceVolts = 3.3;
const float R1 = 4700; 
const float R2 = 1000;
const float resistorFactor = 1023.0 * (R2/(R1 + R2));
const int batteryPin = A4;
int val;
float volts;

/*****Setup******/
void setup()
{
 Serial.begin(9600);
}

/*****Loop******/
void loop()
{
 val = analogRead(batteryPin);
 volts = (val / resistorFactor) * referenceVolts; // calculate the ratio 
 Serial.print("Voltage value : ");
 Serial.println(volts); 
 delay(2000);
}
