// Test sketch for slave device for I2C communication
// Examples by Nicholas Zambetti used as reference
// <https://www.arduino.cc/en/Tutorial/MasterWriter>
// <https://www.arduino.cc/en/Tutorial/MasterReader>

//#include <ADC.h>
#include <Wire.h>
#include <Servo.h>

// Adjust the I2C ID as needed
const byte moduleID = 16;

// Servo object and values
const byte servoPin = 9;
Servo servo;
byte rxInt;
byte rxDec;
float angle = 90.0;
int mappedAngle = 1500;

// Analog samples
//const byte posPin = A2;
//const byte torqPin = A9;
uint16_t rawPosVal = 5500;
uint16_t rawTorqVal = 18000;
byte tx1 = (rawPosVal & 0xFF);
byte tx2 = (rawPosVal >> 8);
byte tx3 = (rawTorqVal & 0xFF);
byte tx4 = (rawTorqVal >> 8);

void setup() {
  Serial.begin(9600);
  //pinMode(posPin, INPUT);
  //pinMode(torqPin, INPUT);
  
  Wire.begin(moduleID);
  Wire.onRequest(sendValues);
  Wire.onReceive(receiveAngles);
  
  servo.attach(servoPin);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop()
{
  //rawPosVal = analogRead(
}

// Function executed upon receiving angle from master device
void receiveAngles(int num)
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

void sendValues()
{
  Wire.write(tx1);
  Wire.write(tx2);
  Wire.write(tx3);
  Wire.write(tx4);
}

