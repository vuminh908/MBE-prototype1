// Test sketch for slave device for I2C communication
// Examples by Nicholas Zambetti used as reference
// <https://www.arduino.cc/en/Tutorial/MasterWriter>
// <https://www.arduino.cc/en/Tutorial/MasterReader>

#include <Wire.h>
#include <Servo.h>

// Adjust the I2C ID as needed
const byte moduleID = 8;

// Servo object and values
const byte servoPin = 9;
Servo servo;
byte rxInt;
byte rxDec;
float angle = 90.0;
int mappedAngle = 1500;

void setup() {
  Serial.begin(9600);
  
  Wire.begin(moduleID);
  Wire.onReceive(receiveData);
    
  servo.attach(servoPin);
}

void loop()
{
  //delay(100);
}

// Function executed upon receiving angle from master device
void receiveData()
{
  rxInt = Wire.read();
  rxDec = Wire.read();
  angle = rxInt + (rxDec / 10.0);
  mappedAngle = map(angle, 0, 180, 1000, 2000);

  Serial.print(angle);
  Serial.print('\t');
  Serial.println(mappedAngle);
  
  servo.writeMicroseconds(mappedAngle);
  
  //delay(15);
}

