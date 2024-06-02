/* 
  Yet Another Hydroponics Project (YAHP) - EXTENSION 1
  
  Author: Riccardo Finotello
  
  A first project using a ESP32 Dev Module. A simple hydroponics capable to decide
  whether to irrigate our poor plants.
  
  Arduino IoT cloud reference sketch: https://create.arduino.cc/cloud/things/c2d3b497-95c4-4825-b029-9a731322f7d5 

  Arduino IoT Cloud Variables description

  The following variables are automatically generated and updated when changes are made to the Thing

  CloudPercentage day_intensity;
  CloudRelativeHumidity moist_4;
  CloudSchedule onScheduler;

  Variables which are marked as READ/WRITE in the Cloud Thing will also have functions
  which are called when their values are changed from the Dashboard.
  These functions are generated with the Thing and added at the end of this sketch.
*/

#include "thingProperties.h"

const int MOIST_4 = 32;  // pins of the moisture sensor
const int MOIST_THRESH_DRY = 60;  // threshold for watering (%)
const int MOIST_THRESH_WET = 90;  // threshold for watering (%)

const int LEDPIN_RED = 17;  // led pin
const int LEDPIN_GREEN = 16;  // led pin
int value_red = 0;  // intensity red pin
int value_green = 0;  // intensity green pin
bool ledIncrease = true;

int wait = 0;  // wait interval
int led_wait = 0;  // led wait interval

const int WAIT_THRESH = 10*1000;  // serial console and sensor update waiting time (ms)
const int LED_THRESH = 15;  // wait threshold to update the leds (ms)

void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500); 

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
 */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  // Initialize YAHP
  Serial.print("\n");
  Serial.print("*************************\n");
  Serial.print("    Initializing YAHP    \n");
  Serial.print("       EXTENSION 1       \n");
  Serial.print("                         \n");
  Serial.print("    auth: thesfinox      \n");
  Serial.print("    ver:  1.0.0          \n");
  Serial.print("*************************\n\n");
  
  pinMode(LEDPIN_RED, OUTPUT);
  pinMode(LEDPIN_GREEN, OUTPUT);
  pinMode(MOIST_4, INPUT);

  // Run tests
  Serial.println("Launching tests...");
  testOutputPin(LEDPIN_RED);
  testOutputPin(LEDPIN_GREEN);
  
  delay(3000);

  // Start collecting data
  Serial.println("HEADER");
  Serial.println("moist_4");
  Serial.println("HEADER");
}

void loop() {
  ArduinoCloud.update();
  if (millis() - wait > WAIT_THRESH)
  {
    wait = millis();
    moist_4 = map(4095 - analogRead(MOIST_4), 0, 4095, 0, 100);
    moist_4 = calibrate(moist_4, 37, 85);
    Serial.print(moist_4);

    Serial.print("\n");
  }  

  if ((day_intensity > 0.0) && onScheduler.isActive())
  {
    if (millis() - led_wait > LED_THRESH)
    {
      led_wait = millis();
  
      // Increase or decrease LED
      if (moist_4 < MOIST_THRESH_DRY)
      {
        value_red = ledBrightness(value_red);
        value_green = 0;
      } else if (moist_4 >= MOIST_THRESH_WET)
      {
        value_green = ledBrightness(value_green);
        value_red = 0;
      } else
      {
        value_green = 0;
        value_red = 0;
      }
      analogWrite(LEDPIN_GREEN, value_green);
      analogWrite(LEDPIN_RED, value_red);
    }
  }
}

void testOutputPin(int pin)  // test a particular pin by making it blink
{
  digitalWrite(pin, HIGH);
  delay(3000);
  digitalWrite(pin, LOW);
  delay(1500);
}

float calibrate(float value, float low, float high)  // recalibrate values between 0 and 100
{
  if (value < low) {return 0.0;}
  if (value >= high) {return 100.0;}
  return 100.0 * (value - low) / (high - low);
}

int ledBrightness(int value)
{
  if ((value == -1) || (value == 75))
  {
    ledIncrease = !ledIncrease;
  }
  if (ledIncrease)
  {
    return value + 1;
  } else
  {
    return value - 1;
  }
}


/*
  Since DayIntensity is READ_WRITE variable, onDayIntensityChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onDayIntensityChange()  {}

/*
  Since OnScheduler is READ_WRITE variable, onOnSchedulerChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onOnSchedulerChange()  {
  // Add your code here to act upon OnScheduler change
}
