// Serial port read:
//   Code from user Robin2 on Arudino Forum used as reference
// Interfacing with ADS1115 ADC+PGA:
//   Example code from ADS1x15 documentation (by Bill Earl and Tony DiCola) used as reference

#include <Servo.h>
#include <Wire.h>
#include "Adafruit_ADS1015.h"

// Master device's own servo
Servo servo1;
const byte servo1Pin = 9;


// I2C IDs of link microcontrollers
const byte moduleID2 = 8;
const byte moduleID3 = 16;
const byte moduleID4 = 24;
const byte moduleID5 = 32;

// ADC objects (up to 4 using the ADS1115)
Adafruit_ADS1115 adc1; // Default address is 0x48
Adafruit_ADS1115 adc2(0x49);
//Adafruit_ADS1115 adc3(0x4A);
//Adafruit_ADS1115 adc4(0x4B);
//Adafruit_ADS1115 adc5(0x4B);

// ADC pins
const byte torqADCPin = 0;
const byte posADCPin = 1;

// Values received from ADCs
int16_t rawTorq1;
int16_t rawTorq2;
int16_t rawTorq3;
int16_t rawTorq4;
//int16_t rawTorq5;

int16_t rawPos1;
int16_t rawPos2;
int16_t rawPos3;
int16_t rawPos4;
//int16_t rawPos5;

// Values for parsing input from serial port
const byte numChars = 7;
char input1[numChars];
char input2[numChars];
char input3[numChars];
char input4[numChars];
char input5[numChars];
boolean newData  = false;
const char startMarker1 = 'a';
const char startMarker2 = 'b';
const char startMarker3 = 'c';
const char startMarker4 = 'd';
const char startMarker5 = 'e';
const char endMarker   = '\r';

// Angle values for sending to links
byte x1;
byte x2;
byte x3;
byte x4;
byte x5;
float xpwm1;

void setup()
{
  //Serial.begin(115200);
  Serial.begin(9600);

  Wire.begin();

  adc1.begin();
  adc2.begin();
  //adc3.begin();
  //adc4.begin();
  //adc5.begin();
  
  servo1.attach(servo1Pin);
} // End setup function


void loop()
{
  recieveData();

  if(newData == true)
  {
    
    x1 = atoi(input1);
    xpwm1 = map(x1, 0, 180, 1000, 2000);
    
    sendData();
    servo1.writeMicroseconds(xpwm1);
    /*
    Serial.print(x1);
    Serial.print("\t");
    Serial.print(x2);
    Serial.print("\t");
    Serial.print(x3);
    Serial.print("\t");
    Serial.print(x4);
    Serial.print("\t");
    Serial.println(x5);
    */
    newData = false;
  }

  readServoData();
  
  Serial.print(rawTorq1);
  Serial.print("\t");
  Serial.print(rawTorq2);
  Serial.println("\t");
  /*Serial.print(rawTorq3);
  Serial.print("\t");
  Serial.print(rawTorq4);
  Serial.print("\t");
  Serial.println(rawTorq5);
  /*
  Serial.print(rawPos2);
  Serial.print("\t");
  Serial.print(rawPos3);
  Serial.print("\t");
  Serial.print(rawPos4);
  Serial.print("\t");
  Serial.println(rawPos5);
  */
  //delay(15);
} // End loop function


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
              if (ndx1 >= numChars)
              {
                ndx1 = numChars - 1;
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
              if (ndx2 >= numChars)
              {
                ndx2 = numChars - 1;
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
              if (ndx3 >= numChars)
              {
                ndx3 = numChars - 1;
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
              if (ndx4 >= numChars)
              {
                ndx4 = numChars - 1;
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
            if (rc != endMarker)
            {
              input5[ndx5] = rc;
              ndx5++;
              if (ndx5 >= numChars)
              {
                ndx5 = numChars - 1;
              }
            }
            else // rc is endMarker, finish reading
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


void sendData()
{
  x2 = atoi(input2);
  x3 = atoi(input3);
  x4 = atoi(input4);
  x5 = atoi(input5);

  Wire.beginTransmission(moduleID2);
  Wire.write(x2);
  Wire.endTransmission();
  /*
  Wire.beginTransmission(moduleID3);
  Wire.write(x3);
  Wire.endTransmission();
  /*
  Wire.beginTransmission(moduleID4);
  Wire.write(x4);
  Wire.endTransmission();

  Wire.beginTransmission(moduleID5);
  Wire.write(x5);
  Wire.endTransmission();
  */
} // End sendData function

void readServoData()
{
  rawTorq1 = adc1.readADC_SingleEnded(torqADCPin);
  rawTorq2 = adc2.readADC_SingleEnded(torqADCPin);
  /*
  rawTorq3 = adc3.readADC_SingleEnded(torqADCPin);
  rawTorq4 = adc4.readADC_SingleEnded(torqADCPin);
  rawTorq5 = adc5.readADC_SingleEnded(torqADCPin);
  
  rawPos1 = adc1.readADC_SingleEnded(posADCPin);
  rawPos2 = adc2.readADC_SingleEnded(posADCPin);
  /*
  rawPos3 = adc3.readADC_SingleEnded(posADCPin);
  rawPos4 = adc4.readADC_SingleEnded(posADCPin);
  rawPos5 = adc5.readADC_SingleEnded(posADCPin);
  */
} // End readServoData function

