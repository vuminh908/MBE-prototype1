// Test sketch for slave device for I2C communication
// Examples by Nicholas Zambetti used as reference
// <https://www.arduino.cc/en/Tutorial/MasterWriter>
// <https://www.arduino.cc/en/Tutorial/MasterReader>

#include <Wire.h>
#include <Servo.h>

// Adjust this value as needed
const byte moduleID = 8;

const byte servoPin = 9;
Servo servo;
byte angle = 0;

const byte potentiometerPin = 0;
int rawPosVal;

void setup() {
  Wire.begin(moduleID);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.begin(9600);
  servo.attach(servoPin);
}

void loop()
{
  rawPosVal = analogRead(potentiometerPin);
  //Serial.println(rawPosVal);
}

void receiveData()
{
  /*
  while(Wire.available() > 0)
  {
    char c = Wire.read();
    Serial.print(c);
  }
  Serial.println();
  */
  //float data = Wire.read();
  //Serial.println(data);
  
  angle = Wire.read();
  float mappedAngle = map(angle, 0, 180, 1000, 2000);

  Serial.print(angle);
  Serial.print('\t');
  Serial.println(mappedAngle);
  servo.writeMicroseconds(mappedAngle);
  //delay(15);
  
}

void sendData()
{
  // rawPosVal is split into 2 bytes for transmission
  // Least significant byte is sent last
  byte b1 = (rawPosVal >> 8);
  byte b2 = (0xFF & rawPosVal);
  Wire.write(b1);
  Wire.write(b2);
}

