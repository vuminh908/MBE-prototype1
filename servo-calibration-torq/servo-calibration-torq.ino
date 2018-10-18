// Input 'a' followed by the desired servo angle into the Serial monitor
// Teensy outputs the raw ADC value (representing torque)

#include <Servo.h>
#include <ADC.h>

Servo servo;
const byte servoPin = 9;
const byte angle = 90; // Keep servo at constant angle of 90 degrees
int   anglePWM = 1558; // Keep servo at constant angle of 90 degrees
// Servo 1: anglePWM = 1545;
// Servo 2: anglePWM = 1580;
ADC *adc = new ADC();
const byte adcPin = A2;
uint16_t adcReading; // In this case, raw torque value

unsigned long usDelay = 10000; // Sampling delay, in microseconds
unsigned long timeStamp;

const byte numCharsIn = 16;
char inputArr[numCharsIn];
boolean newData  = false;
const char startMarkerIn = 'a';
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
    usDelay = atol(inputArr);
    //Serial.println(usDelay);

    newData = false;
  }

 // servo.write(angle);
  servo.writeMicroseconds(anglePWM);

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
  static enum {NOREAD, READ} readState = NOREAD;

  static char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (readState != NOREAD)
    {
      if (rc != endMarkerIn)
      {
        inputArr[ndx] = rc;
        ndx++;
        if (ndx >= numCharsIn)
        {
          ndx = numCharsIn - 1;
        }
      }
      else // rc is endMarker, finish reading
      {
        readState = NOREAD;
        inputArr[ndx] = '\0';
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
