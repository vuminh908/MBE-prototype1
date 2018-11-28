// Serial port read:
//   Code from user Robin2 on Arudino Forum used as reference
// Interfacing with Teensy ADC:
//   Examples by pedvide, author of Teensy ADC library, used as reference

#include <i2c_t3.h>

// Master I2C address
const byte masterID  = 0;

// I2C pins
const byte sclPin = 19;
const byte sdaPin = 18;

// I2C addresses of link microcontrollers
const byte moduleID1 = 8;
const byte moduleID2 = 16;
const byte moduleID3 = 24;
const byte moduleID4 = 32;
const byte moduleID5 = 40;

// Number of links (at least 1)
byte numLinks = 1;

// Values received from ADCs of slave devices
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
const char startMarkerN = 'n';

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

// Angle values for sending to links
float angle1;
float angle2;
float angle3;
float angle4;
float angle5;

void setup()
{
  Serial.begin(115200);
  //Serial.begin(9600);

  // I2C communication
  Wire.begin(I2C_MASTER, masterID, sclPin, sdaPin); // Default external pullups and 400kHz
  Wire.setDefaultTimeout(200000); // Set timeout value to 200ms

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
    /*
    Serial.print(rawPos1);
    Serial.print('\t');
    Serial.print(rawPos2);
    Serial.print('\t');
    Serial.print(rawPos3);
    Serial.print('\t');
    Serial.print(rawPos4);
    Serial.print('\t');
    Serial.print(rawPos5);
    Serial.print("\t\t");
    /**/
    /*
    Serial.print(rawTorq1);
    Serial.print('\t');
    Serial.print(rawTorq2);
    Serial.print('\t');
    Serial.print(rawTorq3);
    Serial.print('\t');
    Serial.print(rawTorq4);
    Serial.print('\t');
    Serial.println(rawTorq5);
    /**/
    
    reportBack();
        
    sendData();
    
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
// Reads value up to four significant digits (with decimal point)
void recieveData()
{
  static byte ndx1 = 0;
  static byte ndx2 = 0;
  static byte ndx3 = 0;
  static byte ndx4 = 0;
  static byte ndx5 = 0;  
  static enum {NOREAD, READ1, READ2, READ3, READ4, READ5, READN} readState = NOREAD;

  static char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    // State machine for parsing input string from serial port
    if (readState != NOREAD)
    {

      switch (readState)
      {
        case READN: // Read new link number (should be a single value)
          {
            if(rc != endMarkerIn)
            {
              // rc should be a character representing a value between 1 and 9 in ASCII
              byte tempNum = rc - 0x30;
              if(tempNum >= 1 && tempNum <= 9)
              {
                numLinks = tempNum;
              }
              else // Erroneous link number, don't modify numLinks, drop attempted read and move on
              {
                readState = NOREAD;
              }
            }
            else // rc is endMarkerIn, finish reading
            {
              readState = NOREAD;
            }
          }
        case READ1: // Read angle for servo 1
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
            else // rc is startMarker2
            {
              input1[ndx1] = '\0';
              ndx1 = 0;
              if(numLinks >= 2)
              {
                // Start reading servo 2 angle
                readState = READ2;
              }
              else
              {
                readState = NOREAD;
                newData = true;
              }
            }
            break;
          }
        case READ2: // Read angle for servo 2
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
            else // rc is startMarker3
            {
              input2[ndx2] = '\0';
              ndx2 = 0;
              if(numLinks >= 3)
              {
                // Start reading servo 3 angle
                readState = READ3;
              }
              else
              {
                readState = NOREAD;
                newData = true;
              }
            }
            break;
          }
        case READ3: // Read angle for servo 3
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
            else // rc is startMarker4
            {
              input3[ndx3] = '\0';
              ndx3 = 0;
              if(numLinks >= 4)
              {
                // Start reading servo 4 angle
                readState = READ4;
              }
              else
              {
                readState = NOREAD;
                newData = true;
              }
            }
            break;
          }
        case READ4: // Read angle for servo 4
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
            else // rc is startMarker5
            {
              input4[ndx4] = '\0';
              ndx4 = 0;
              if(numLinks >= 5)
              {
                // Start reading servo 5 angle
                readState = READ5;
              }
              else
              {
                readState = NOREAD;
                newData = true;
              }
            }
            break;
          }                    
        case READ5: // Read angle for servo 5
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
              input5[ndx5] = '\0';
              ndx5 = 0;
              readState = NOREAD;
              newData = true;
            }
            break;
          }          
        default: // Should not reach here
          {
            // Just in case, reset everything
            readState = NOREAD;
            ndx1 = 0;
            ndx2 = 0;
            ndx3 = 0;
            ndx4 = 0;
            ndx5 = 0;
            break;
          }
      }
    }
    else if(rc == startMarkerN)  // Begin reading link number config string
    {
      readState = READN;
    }
    else if (rc == startMarker1) // Begin reading servo angles string
    {
      readState = READ1;
    }

  } // End while loop

} // End recieveData function


// Send servo angles to each link
void sendData()
{
  // Split into two bytes for transmission - integer and decimal portions
  byte txInt1;
  byte txDec1;
  byte txInt2;
  byte txDec2;
  byte txInt3;
  byte txDec3;
  byte txInt4;
  byte txDec4;
  byte txInt5;
  byte txDec5;

  // Can safely assume at least 1 link
  angle1 = atof(input1);
  txInt1 = (byte)angle1;
  txDec1 = (byte)((int)(angle1 * 10) % 10);
  Wire.beginTransmission(moduleID1);
  Wire.write(txInt1);
  Wire.write(txDec1);
  Wire.endTransmission();

  if(numLinks >= 2)
  {
    angle2 = atof(input2);
    txInt2 = (byte)angle2;
    txDec2 = (byte)((int)(angle2 * 10) % 10);
    Wire.beginTransmission(moduleID2);
    Wire.write(txInt2);
    Wire.write(txDec2);
    Wire.endTransmission();
  }
  
  if(numLinks >= 3)
  {
    angle3 = atof(input3);
    txInt3 = (byte)angle3;
    txDec3 = (byte)((int)(angle3 * 10) % 10);
    Wire.beginTransmission(moduleID3);
    Wire.write(txInt3);
    Wire.write(txDec3);
    Wire.endTransmission();
  }

  if(numLinks >= 4)
  {
    angle4 = atof(input4);
    txInt4 = (byte)angle4;
    txDec4 = (byte)((int)(angle4 * 10) % 10);
    Wire.beginTransmission(moduleID4);
    Wire.write(txInt4);
    Wire.write(txDec4);
    Wire.endTransmission();
  }

  if(numLinks >= 5)
  {
    angle5 = atof(input5);
    txInt5 = (byte)angle5;
    txDec5 = (byte)((int)(angle5 * 10) % 10);
    Wire.beginTransmission(moduleID5);
    Wire.write(txInt5);
    Wire.write(txDec5);
    Wire.endTransmission();
  }
  
} // End sendData function


// Read raw servo data (position and torque) from links
void readServoData()
{
  byte rx1;
  byte rx2;
  byte rx3;
  byte rx4;

  // Can safely assume at least 1 link
  Wire.requestFrom(moduleID1, 4);
  rx1 = Wire.read();
  rx2 = Wire.read();
  rx3 = Wire.read();
  rx4 = Wire.read();
  rawPos1 = (rx2 << 8) + rx1;
  rawTorq1 = (rx4 << 8) + rx3;

  if(numLinks >= 2)
  {
    Wire.requestFrom(moduleID2, 4);
    rx1 = Wire.read();
    rx2 = Wire.read();
    rx3 = Wire.read();
    rx4 = Wire.read();
    rawPos2 = (rx2 << 8) + rx1;
    rawTorq2 = (rx4 << 8) + rx3;
  }
  else
  {
    rawPos2 = 0;
    rawTorq2 = 0;
  }
  
  if(numLinks >= 3)
  {
    Wire.requestFrom(moduleID3, 4);
    rx1 = Wire.read();
    rx2 = Wire.read();
    rx3 = Wire.read();
    rx4 = Wire.read();
    rawPos3 = (rx2 << 8) + rx1;
    rawTorq3 = (rx4 << 8) + rx3;
  }
  else
  {
    rawPos3 = 0;
    rawTorq3 = 0;
  }

  if(numLinks >= 4)
  {
    Wire.requestFrom(moduleID4, 4);
    rx1 = Wire.read();
    rx2 = Wire.read();
    rx3 = Wire.read();
    rx4 = Wire.read();
    rawPos4 = (rx2 << 8) + rx1;
    rawTorq4 = (rx4 << 8) + rx3;
  }
  else
  {
    rawPos4 = 0;
    rawTorq4 = 0;
  }

  if(numLinks >= 5)
  {
    Wire.requestFrom(moduleID5, 4);
    rx1 = Wire.read();
    rx2 = Wire.read();
    rx3 = Wire.read();
    rx4 = Wire.read();
    rawPos5 = (rx2 << 8) + rx1;
    rawTorq5 = (rx4 << 8) + rx3;
  }
  else
  {
    rawPos5 = 0;
    rawTorq5 = 0;
  }
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

