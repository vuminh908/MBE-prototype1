// Input 'a' followed by the desired servo angle into the Serial monitor
// Teensy outputs the raw ADC value (representing torque)

#include <Servo.h>
#include <ADC.h>

Servo servo;
const byte servoPin = 9;
ADC *adc = new ADC();
int usPWM; // Microseconds value received from serial input

const byte torqPin = A2;
const byte posPin = A9;
uint16_t rawTorq; // Raw ADC value of current sensor output (proportional to torque)
uint16_t rawPos;  // Raw ADC value of feedback potentiometer voltage

unsigned long usDelay = 10000; // Sampling delay, in microseconds
unsigned long timeStamp;

const byte numCharsIn = 4;
char inputArr[numCharsIn];
boolean newData  = false;
const char startMarkerIn = 'a';
const char endMarkerIn = '\r';

//const char startMarkerOut = 'a';
//const char endMarkerOut = '!';
String outputStr;
//const byte numCharsOut = 64;
//char outputArr[numCharsOut];

void setup()
{
  Serial.begin(115200);
  //Serial.begin(9600);

  pinMode(torqPin, INPUT);
  pinMode(posPin, INPUT);
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
    usPWM = atoi(inputArr);
    newData = false;
  }

  //servo.write(angle);
  servo.writeMicroseconds(usPWM);

  if((micros() - timeStamp) >= usDelay)
  {
    rawTorq = adc->analogRead(torqPin);
    rawPos = adc->analogRead(posPin);
    
    outputStr = "Torque: " + String(rawTorq) + "\tPosition: " + String(rawPos) + "\r\n";
    //outputStr.toCharArray(outputArr, numCharsOut);
    //Serial.write(outputArr);
    Serial.println(outputStr);

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
