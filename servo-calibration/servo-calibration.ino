// Input 'a' followed by the desired servo angle into the Serial monitor
// Arduino interfaces with ADS1115 ADC through I2C

#include <Servo.h>
#include <Wire.h>
#include "Adafruit_ADS1015.h"

Servo myservo;
const byte servoPin = 9;

//Adafruit_ADS1115 adc; // Default address is 0x48

int us; // Number of microseconds to write to servo
int adcReading;

const byte numChars = 7;
char input[numChars];
boolean newData  = false;
char startMarkerIn = 'a';
char endMarkerIn = '\r';
char startMarkerOut = 'a';
char endMarkerOut = '!';
String outputStr;
char output[numChars];

void setup()
{
  //Serial.begin(115200);
  Serial.begin(9600);

  myservo.attach(servoPin);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  //adc.begin();
} // End setup function

void loop()
{
  recieveData();

  if(newData == true)
  {    
    us = atof(input);
    
    myservo.writeMicroseconds(us);

    newData = false;
  }
  /*
  adcReading = adc.readADC_SingleEnded(0);

  //outputStr = startMarkerOut + String(adcReading) + endMarkerOut;
  outputStr = startMarkerOut + String(us) + endMarkerOut;
  outputStr.toCharArray(output, numChars);
  Serial.write(output);
  //Serial.println(outputStr);
  */
  //delay(50);

} // End loop function



void recieveData()
{
  static byte ndx = 0;
  static enum {NOREAD, READ} readState = NOREAD;

  static char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (readState != NOREAD)
    {
      if (rc != endMarkerIn)
      {
        input[ndx] = rc;
        ndx++;
        if (ndx >= numChars)
        {
          ndx = numChars - 1;
        }
      }
      else // rc is endMarker, finish reading
      {
        readState = NOREAD;
        input[ndx] = '\0';
        ndx = 0;
        newData = true;
      }      
    }
    else if (rc == startMarkerIn) // Begin reading
    {
      readState = READ;
    }

  } // End while loop

} // End recieveData function
