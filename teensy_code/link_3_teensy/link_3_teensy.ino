// Test sketch for slave device for I2C communication
// Example sketches used for reference:
//   I2C examples by Nicholas Zambetti
//    <https://www.arduino.cc/en/Tutorial/MasterWriter>
//    <https://www.arduino.cc/en/Tutorial/MasterReader>
//   ADC examples for using Teensy ADC library by pedvide
//    <https://github.com/pedvide/ADC/blob/master/examples/analogRead/analogRead.ino>
//    <https://github.com/pedvide/ADC/blob/master/examples/readPin/readPin.ino>
//    <https://github.com/pedvide/ADC/blob/master/examples/synchronizedMeasurements/synchronizedMeasurements.ino>

#include <ADC.h>
#include <Servo.h>
#include <i2c_t3.h>

// I2C address
const byte moduleID = 24;

// I2C pins
const byte sclPin = 19;
const byte sdaPin = 18;

// Receive and transmit function prototypes
void receiveAngles(size_t num);
void sendValues();

// Servo
const int minAngle = 70;
const int maxAngle = 110;
const int minMicrosec = 1260;  // Adjust min and max as needed when calibrating servo
const int maxMicrosec = 1670;
const byte servoPin = 9;
Servo servo;
byte rxInt;
byte rxDec;
float angle = 90.0;
int mappedAngle = 1500;

// Analog sampling
ADC *adc = new ADC();
ADC::Sync_result result;
const byte posPin = A9;  // Corresponds to ADC0
const byte torqPin = A2; // Corresponds to ADC1
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
  adc->setAveraging(4, ADC_0);
  adc->setResolution(16, ADC_0);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED, ADC_0);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED, ADC_0);

  // ADC1
  adc->setAveraging(4, ADC_1);
  adc->setResolution(16, ADC_1);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED, ADC_1);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED, ADC_1);

  // I2C communication
  Wire.begin(I2C_SLAVE, moduleID, sclPin, sdaPin);
  Wire.onRequest(sendValues);
  Wire.onReceive(receiveAngles);

  // Servo
  servo.attach(servoPin, minMicrosec, maxMicrosec);

  // Turn on built-in LED so we know the Teensy is on and the setup completed
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
} // End setup function

void loop()
{
  /*
  // Single reads
  rawPosVal = adc->analogRead(posPin, ADC_0);
  rawTorqVal = adc->analogRead(torqPin, ADC_1);
  /**/
  /**/
  // Synchronized read
  result = adc->analogSyncRead(posPin, torqPin);
  rawPosVal = result.result_adc0;
  rawTorqVal = result.result_adc1;
  /**/
  /*
  // Debugging
  Serial.print(rawPosVal);
  Serial.print('\t');
  Serial.println(rawTorqVal);
  adc->printError();
  /**/
  //delay(50);
} // End loop function

// Function executed upon receiving angle from master device
void receiveAngles(size_t num)
{
  rxInt = Wire.read();
  rxDec = Wire.read();
  angle = rxInt + (rxDec / 10.0);
  mappedAngle = map(angle, minAngle, maxAngle, minMicrosec, maxMicrosec);

  Serial.print(angle);
  Serial.print('\t');
  Serial.println(mappedAngle);

  //servo.write(angle);
  servo.writeMicroseconds(mappedAngle);
} // End receiveAngles function

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
} // End sendValues function

