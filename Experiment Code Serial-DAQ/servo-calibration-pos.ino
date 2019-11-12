// Input 'a' followed by the desired servo angle into the Serial monitor
// Teensy outputs the raw ADC value (representing position)

#include <Servo.h>
#include <ADC.h>

Servo servo;
const byte servoPin = 9;
int usPWM; // Microseconds value received from LabVIEW

ADC *adc = new ADC();
const byte adcPin = A9;
uint16_t adcReading; // In this case, raw position value

unsigned long usDelay = 10000; // Sampling delay, in microseconds
unsigned long timeStamp;

const byte numCharsIn = 16;
char inputArr1[numCharsIn];
char inputArr2[numCharsIn];
boolean newData  = false;
const char startMarkerIn1 = 'a';
const char startMarkerIn2 = 'b';
const char endMarkerIn = '\r';
const char startMarkerOut = 'a';
const char endMarkerOut = '!';
String outputStr;
const byte numCharsOut = 8;
char outputArr[numCharsOut];

void setup()
{
  Serial.begin(115200);
  //Serial.begin(9600);

  pinMode(adcPin, INPUT);
  adc->setAveraging(16);
  adc->setResolution(16);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  
  servo.attach(servoPin);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  timeStamp = micros();
} // End setup function

void loop()
{
  recieveData();

  if(newData == true)
  {    
    usDelay = atol(inputArr1);
    usPWM = atoi(inputArr2);
    //Serial.print(usDelay);
    //Serial.print('\t');
    //Serial.println(usPWM);

    servo.writeMicroseconds(usPWM);
  
    newData = false;
  }
  
  if((micros() - timeStamp) >= usDelay)
  {
    adcReading = adc->analogRead(adcPin);
    
    outputStr = startMarkerOut + String(adcReading) + endMarkerOut;
    outputStr.toCharArray(outputArr, numCharsOut);
    Serial.write(outputArr);
    //Serial.println(outputStr);

    timeStamp = micros();
  }
    
} // End loop function



void recieveData()
{
  static byte ndx = 0;
  static enum {NOREAD, READ1, READ2} readState = NOREAD;

  static char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (readState != NOREAD)
    {
      switch(readState)
      {
        case READ1:
        {
          if (rc != startMarkerIn2)
            {
              inputArr1[ndx] = rc;
              ndx++;
              if (ndx >= numCharsIn)
              {
                ndx = numCharsIn - 1;
              }
            }
            else // rc is startMarker2, start reading second value
            {
              readState = READ2;
              inputArr1[ndx] = '\0';
              ndx = 0;
            }
            break;
        }
        case READ2:
        {
          if (rc != endMarkerIn)
          {
            inputArr2[ndx] = rc;
            ndx++;
            if (ndx >= numCharsIn)
            {
              ndx = numCharsIn - 1;
            }
          }
          else // rc is endMarker, finish reading
          {
            readState = NOREAD;
            inputArr2[ndx] = '\0';
            ndx = 0;
            newData = true;
          }
        }
      }
    }
    else if (rc == startMarkerIn1) // Begin reading first value
    {
      readState = READ1;
    }

  } // End while loop

} // End recieveData function
