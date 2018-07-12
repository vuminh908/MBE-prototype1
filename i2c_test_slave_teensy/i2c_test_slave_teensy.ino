// Test sketch for slave device for I2C communication
// Example sketches used for reference:
//   I2C examples by Nicholas Zambetti
//    <https://www.arduino.cc/en/Tutorial/MasterWriter>
//    <https://www.arduino.cc/en/Tutorial/MasterReader>
//   ADC examples for using Teensy ADC library by pedvide
//    <https://github.com/pedvide/ADC/blob/master/examples/analogRead/analogRead.ino>
//    <https://github.com/pedvide/ADC/blob/master/examples/readPin/readPin.ino>

#include <ADC.h>
#include <Wire.h>
#include <Servo.h>

// Adjust the I2C ID as needed
const byte moduleID = 16;

// Servo
const byte servoPin = 9;
Servo servo;
byte rxInt;
byte rxDec;
float angle = 90.0;
int mappedAngle = 1500;

// Analog sampling
ADC *adc = new ADC();
const byte posPin = A2;  // Corresponds to ADC0
const byte torqPin = A9; // Corresponds to ADC1
uint16_t rawPosVal;
uint16_t rawTorqVal;
byte tx1;
byte tx2;
byte tx3;
byte tx4;

void setup()
{
  Serial.begin(9600); // For debugging
  // Analog pins
  pinMode(posPin, INPUT);
  pinMode(torqPin, INPUT);

  // ADC0
  adc->setAveraging(16, ADC_0);
  adc->setResolution(16, ADC_0);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED, ADC_0);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED, ADC_0);

  // ADC1
  adc->setAveraging(16, ADC_1);
  adc->setResolution(16, ADC_1);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED, ADC_1);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED, ADC_1);

  // I2C communication
  Wire.begin(moduleID);
  Wire.onRequest(sendValues);
  Wire.onReceive(receiveAngles);

  // Servo
  servo.attach(servoPin);

  // Turn on built-in LED so we know the Teensy is on and the setup completed
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop()
{
  rawPosVal = adc->adc0->analogRead(posPin);
  rawTorqVal = adc->adc1->analogRead(torqPin);
  
  //delay(100);
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
}

// Function executed upon master device requesting to read values
void sendValues()
{
  tx1 = (rawPosVal & 0xFF);
  tx2 = (rawPosVal >> 8);
  tx3 = (rawTorqVal & 0xFF);
  tx4 = (rawTorqVal >> 8);
  
  Wire.write(tx1);
  Wire.write(tx2);
  Wire.write(tx3);
  Wire.write(tx4);
}

