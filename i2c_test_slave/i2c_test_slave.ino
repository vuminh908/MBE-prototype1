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
byte angle = 0;
float mappedAngle = 0.0;

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
  angle = Wire.read();
  mappedAngle = map(angle, 0, 180, 1000, 2000);

  Serial.print(angle);
  Serial.print('\t');
  Serial.println(mappedAngle);
  
  servo.writeMicroseconds(mappedAngle); // May move this to loop
  
  //delay(15);
}

