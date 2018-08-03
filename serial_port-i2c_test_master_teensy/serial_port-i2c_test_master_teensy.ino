// Serial port read:
//   Code from user Robin2 on Arudino Forum used as reference
// Interfacing with Teensy ADC:
//   Examples by pedvide, author of Teensy ADC library, used as reference

#include <ADC.h>
#include <Servo.h>
#include <i2c_t3.h>


// Master device's own servo
const int minMicrosec = 906;  // Adjust min and max as needed when calibrating servo
const int maxMicrosec = 1900;
Servo servo1;
const byte servo1Pin = 9;

// Master I2C address
const byte masterID  = 0;

// I2C pins
const byte sclPin = 19;
const byte sdaPin = 18;

// I2C addresses of link microcontrollers
const byte moduleID2 = 8;
const byte moduleID3 = 16;
const byte moduleID4 = 24;
const byte moduleID5 = 32;

// Analog sampling
ADC *adc = new ADC();
ADC::Sync_result result;
const byte posPin = A9;  // Corresponds to ADC0
const byte torqPin = A2; // Corresponds to ADC1

// Values received from ADCs of master and slave devices
uint16_t rawTorq1;
uint16_t rawTorq2;
uint16_t rawTorq3;
uint16_t rawTorq4;
uint16_t rawTorq5;

uint16_t rawPos1;
uint16_t rawPos2;
uint16_t rawPos3;
uint16_t rawPos4;
uint16_t rawPos5;

// Values for parsing input from or writing output to serial port
const byte numCharsIn = 6;
char input1[numCharsIn];
char input2[numCharsIn];
char input3[numCharsIn];
char input4[numCharsIn];
char input5[numCharsIn];
boolean newData  = false;
const char startMarker1 = 'a';
const char startMarker2 = 'b';
const char startMarker3 = 'c';
const char startMarker4 = 'd';
const char startMarker5 = 'e';
const char endMarkerIn  = '\r';

// (6 chars * 10 values) + 1 for endmarker + 1 for null terminator
const byte numCharsOut = 62;
const char startMarkerOut1 = 'v';
const char startMarkerOut2 = 'w';
const char startMarkerOut3 = 'x';
const char startMarkerOut4 = 'y';
const char startMarkerOut5 = 'z';
const char endMarkerOut = '!';
String outputStr;
char output[numCharsOut];

// Angle values for master device's servo
float angle1;
float mappedAngle1;

// Angle values for sending to links
float angle2;
float angle3;
float angle4;
float angle5;

void setup()
{
  Serial.begin(115200);
  //Serial.begin(9600);
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
  Wire.begin(I2C_MASTER, masterID, sclPin, sdaPin); // Default external pullups and 400kHz
  Wire.setDefaultTimeout(200000); // Set timeout value to 200ms
  
  // Servo
  servo1.attach(servo1Pin);

  // Turn on built-in LED so we know the Teensy is on and the setup completed
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
} // End setup function


void loop()
{
  recieveData();

  if(newData == true)
  {
    readServoData();
    
    /*Serial.print(rawPos1);
    Serial.print('\t');
    Serial.print(rawPos2);
    Serial.print('\t');
    Serial.print(rawPos3);
    Serial.print("\t\t");
    Serial.print(rawPos4);
    Serial.print('\t');
    Serial.println(rawPos5);
    
    Serial.print(rawTorq1);
    Serial.print('\t');
    Serial.print(rawTorq2);
    Serial.print('\t');
    Serial.println(rawTorq3);
    /*Serial.print('\t');
    Serial.print(rawTorq4);
    Serial.print('\t');
    Serial.println(rawTorq5);
    */
    
    reportBack();

    angle1 = atof(input1);
    mappedAngle1 = map(angle1, 0, 180, minMicrosec, maxMicrosec);
    
    sendData();
    //servo1.write(angle1);
    servo1.writeMicroseconds(mappedAngle1);
    /*
    Serial.print(angle1);
    Serial.print("\t");
    Serial.print(angle2);
    Serial.print("\t");
    Serial.print(angle3);
    Serial.print("\t");
    Serial.print(angle4);
    Serial.print("\t");
    Serial.println(angle5);
    */
    newData = false;
  }

  //delay(15);
} // End loop function


// Receive servo angles from serial port (can input through serial monitor or LabVIEW)
// Reads value up to one decimal place
void recieveData()
{
  static byte ndx1 = 0;
  static byte ndx2 = 0;
  static byte ndx3 = 0;
  static byte ndx4 = 0;
  static byte ndx5 = 0;  
  static enum {NOREAD, READ1, READ2, READ3, READ4, READ5} readState = NOREAD;

  static char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    // State machine for parsing input string from serial port
    if (readState != NOREAD)
    {

      switch (readState)
      {
        case READ1: // Read angle for servo A
          {
            if (rc != startMarker2)
            {
              input1[ndx1] = rc;
              ndx1++;
              if (ndx1 >= numCharsIn)
              {
                ndx1 = numCharsIn - 1;
              }
            }
            else // rc is startMarker2, start reading servo B angle
            {
              readState = READ2;
              input1[ndx1] = '\0';
              ndx1 = 0;
            }
            break;
          }
        case READ2: // Read angle for servo B
          {
            if (rc != startMarker3)
            {
              input2[ndx2] = rc;
              ndx2++;
              if (ndx2 >= numCharsIn)
              {
                ndx2 = numCharsIn - 1;
              }
            }
            else // rc is startMarker3, start reading servo C angle
            {
              readState = READ3;
              input2[ndx2] = '\0';
              ndx2 = 0;
            }
            break;
          }
        case READ3: // Read angle for servo C
          {
            if (rc != startMarker4)
            {
              input3[ndx3] = rc;
              ndx3++;
              if (ndx3 >= numCharsIn)
              {
                ndx3 = numCharsIn - 1;
              }
            }
            else // rc is startMarker4, start reading servo D angle
            {
              readState = READ4;
              input3[ndx3] = '\0';
              ndx3 = 0;
            }
            break;
          }
        case READ4: // Read angle for servo D
          {
            if (rc != startMarker5)
            {
              input4[ndx4] = rc;
              ndx4++;
              if (ndx4 >= numCharsIn)
              {
                ndx4 = numCharsIn - 1;
              }
            }
            else // rc is startMarker5, start reading servo E angle
            {
              readState = READ5;
              input4[ndx4] = '\0';
              ndx4 = 0;
            }
            break;
          }                    
        case READ5: // Read angle for servo E
          {
            if (rc != endMarkerIn)
            {
              input5[ndx5] = rc;
              ndx5++;
              if (ndx5 >= numCharsIn)
              {
                ndx5 = numCharsIn - 1;
              }
            }
            else // rc is endMarkerIn, finish reading
            {
              readState = NOREAD;
              input5[ndx5] = '\0';
              ndx5 = 0;
              newData = true;
            }
            break;
          }          
        default:
          break;
      }

    }
    else if (rc == startMarker1) // Begin reading
    {
      readState = READ1;
    }

  } // End while loop

} // End recieveData function


// Send servo angles to each link
void sendData()
{
  angle2 = atof(input2);
  angle3 = atof(input3);
  angle4 = atof(input4);
  angle5 = atof(input5);

  // Split into two bytes for transmission - integer and decimal portions
  byte txInt2;
  byte txDec2;
  byte txInt3;
  byte txDec3;
  byte txInt4;
  byte txDec4;
  byte txInt5;
  byte txDec5;
  
  txInt2 = (byte)angle2;
  txInt3 = (byte)angle3;
  txInt4 = (byte)angle4;
  txInt5 = (byte)angle5;

  txDec2 = (byte)((int)(angle2 * 10) % 10);
  txDec3 = (byte)((int)(angle3 * 10) % 10);
  txDec4 = (byte)((int)(angle4 * 10) % 10);
  txDec5 = (byte)((int)(angle5 * 10) % 10);

  Wire.beginTransmission(moduleID2);
  Wire.write(txInt2);
  Wire.write(txDec2);
  Wire.endTransmission();
  
  Wire.beginTransmission(moduleID3);
  Wire.write(txInt3);
  Wire.write(txDec3);
  Wire.endTransmission();
  /*
  Wire.beginTransmission(moduleID4);
  Wire.write(txInt4);
  Wire.write(txDec4);
  Wire.endTransmission();

  Wire.beginTransmission(moduleID5);
  Wire.write(txInt5);
  Wire.write(txDec5);
  Wire.endTransmission();
  */
} // End sendData function


// Read raw servo data (position and torque) from ADCs
void readServoData()
{
  // For now, values for servos 4 and 5 have placeholder values

  // Read master device's ADCs
  /*
  // Single reads
  rawPos1 = adc->adc0->analogRead(posPin);
  rawTorq1 = 5500;//adc->adc1->analogRead(torqPin);
  */
  // Synchronized read
  result = adc->analogSyncRead(posPin, torqPin);
  rawPos1 = result.result_adc0;
  rawTorq1 = result.result_adc1;

  byte rx1;
  byte rx2;
  byte rx3;
  byte rx4;

  Wire.requestFrom(moduleID2, 4);
  rx1 = Wire.read();
  rx2 = Wire.read();
  rx3 = Wire.read();
  rx4 = Wire.read();
  rawPos2 = (rx2 << 8) + rx1;
  rawTorq2 = (rx4 << 8) + rx3;
  
  Wire.requestFrom(moduleID3, 4);
  rx1 = Wire.read();
  rx2 = Wire.read();
  rx3 = Wire.read();
  rx4 = Wire.read();
  rawPos3 = (rx2 << 8) + rx1;
  rawTorq3 = (rx4 << 8) + rx3;
  /*
  rawPos3 = 20000;
  rawTorq3 = 30000;
  */
  rawPos4 = 20000;
  rawTorq4 = 30000;
  
  rawPos5 = 20000;
  rawTorq5 = 30000;
} // End readServoData function


// Write servo values back to serial port (can be read in through LabVIEW or serial monitor)
void reportBack()
{
  // Position values with corresponding start markers,
  // then torque values with corresponding start markers (startMarkerOut)
  // and finally the end marker
  outputStr = startMarker1 + String(rawPos1) + startMarker2 + String(rawPos2) +
              startMarker3 + String(rawPos3) + startMarker4 + String(rawPos4) +
              startMarker5 + String(rawPos5) +
              startMarkerOut1 + String(rawTorq1) + startMarkerOut2 + String(rawTorq2) +
              startMarkerOut3 + String(rawTorq3) + startMarkerOut4 + String(rawTorq4) +
              startMarkerOut5 + String(rawTorq5) + endMarkerOut;

  outputStr.toCharArray(output, numCharsOut);
  
  Serial.write(output);
  //Serial.println(outputStr);
}

