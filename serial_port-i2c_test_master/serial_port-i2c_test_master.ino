#include <Servo.h>
#include <Wire.h>
// Receive up to 5 positions
Servo myservo1;
const byte servo1Pin = 9;

const byte moduleID2 = 8;
const byte moduleID3 = 16;
const byte moduleID4 = 24;
const byte moduleID5 = 32;

const byte maxBytesR = 2;
byte receivedBytes2[maxBytesR];
byte receivedBytes3[maxBytesR];
byte receivedBytes4[maxBytesR];
byte receivedBytes5[maxBytesR];

//const byte potentiometerPin = 0;
//int rawPosVal1;

int rawPosVal2;
int rawPosVal3;
int rawPosVal4;
int rawPosVal5;

const byte numChars = 7;
char input1[numChars];
char input2[numChars];
char input3[numChars];
char input4[numChars];
char input5[numChars];
char output[numChars - 1];
byte x1;
byte x2;
byte x3;
byte x4;
byte x5;
float xpwm1;
float xpwm2;
float xpwm3;
float xpwm4;
float xpwm5;
boolean newData  = false;
char startMarker1 = 'a';
char startMarker2 = 'b';
char startMarker3 = 'c';
char startMarker4 = 'd';
char startMarker5 = 'e';
char endMarker   = '\r';

void setup()
{
  //Serial.begin(115200);
  Serial.begin(9600);

  Wire.begin();

  myservo1.attach(servo1Pin);
} // End setup function

void loop()
{
  recieveData();

  if(newData == true)
  {
    x1 = atoi(input1);
    xpwm1 = map(x1, 0, 180, 1000, 2000);
    //myservo1.writeMicroseconds(xpwm1);

    sendData();

    Serial.print(x1);
    Serial.print("\t");
    Serial.print(x2);
    Serial.print("\t");
    Serial.print(x3);
    Serial.print("\t");
    Serial.print(x4);
    Serial.print("\t");
    Serial.println(x5);
    
    newData = false;
  }

  readFromLinks();
  
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
}

void readFromLinks()
{
  Wire.requestFrom(moduleID2, maxBytesR);
  readWireData(receivedBytes2);
  Serial.print(receivedBytes2[0]);
  Serial.print('\t');
  Serial.print(receivedBytes2[1]);
  Serial.print('\t');
  rawPosVal2 = (receivedBytes2[0] << 8) + receivedBytes2[1];
  Serial.println(rawPosVal2);
  /*
  Wire.requestFrom(moduleID3, maxBytesR);
  readWireData(receivedBytes3);
  /*
  Wire.requestFrom(moduleID4, maxBytesR);
  readWireData(receivedBytes4);
  
  Wire.requestFrom(moduleID5, maxBytesR);
  readWireData(receivedBytes5);
  */
}

void readWireData(byte arr[])
{
  byte index = 0;
  while(Wire.available() && index < maxBytesR)
  {
    arr[index] = Wire.read();
    index++;
  }
}

