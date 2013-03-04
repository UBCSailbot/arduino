/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 
 This example code is in the public domain.
 */

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  double sensorValue0 = analogRead(A1);
//  int sensorValue1 = analogRead(A1);
//  int sensorValue2 = analogRead(A2);
//  int sensorValue3 = analogRead(A3);
//  int sensorValue4 = analogRead(A4);
//  int sensorValue5 = analogRead(A5); 
//  // print out the value you read:
  Serial.println(sensorValue0);
  delay(1000);        // delay in between reads for stability
}

int convertTo360 (double sensorValue){
    int result= (int)-1*sensorValue*360/1024;
    if(result < -180){
            result += 360;
          }
     return result;
}
