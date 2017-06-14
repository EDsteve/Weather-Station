/*                  BME280 I2C for MySensors
// this program is for BMEP280 library :Adafriut BME280 
I use some parts from oryginal MySensors PressureSensor for BMP085 module  
To use with MySensors 2.0.0 dev.branch
Also included is battery level
* REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * Version 1.2 - Marek Dajnowicz

Modified by ED:
- aded battery level read directly from VCC on your arduino
*/


// Enable debug prints to serial monitor
#define MY_DEBUG
//#define MY_REPEATER_FEATURE

// Enable and select radio type attached
#define MY_RADIO_NRF24

//#define MY_NODE_ID 33 

//#define MY_PARENT_NODE_ID

#include <SPI.h>
#include <MySensors.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> // I had to change I2C address in library for 0x76 (line 32)
//#undef BME280_ADDRESS         // Undef BME280_ADDRESS from the BME280 library to easily override I2C address
//#define BME280_ADDRESS (0x76) // Low = 0x76 , High = 0x77 (default on adafruit and sparkfun BME280 modules, default for library)
#include "Wire.h"

Adafruit_BME280 bme; // I2C

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_PRESS 2
#define CHILD_ID_BATTERY 3
#define BATTERY_REPORT_CYCLE 2
#define VMIN 3400           // Min Battery level
#define VMAX 4000           // Max Battery level

unsigned long SLEEP_TIME = 120000; // Sleep time between reads (in milliseconds)

const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
  STABLE = 0,      // "Stable Weather Pattern"
  SUNNY = 1,      // "Slowly rising Good Weather", "Clear/Sunny "
  CLOUDY = 2,     // "Slowly falling L-Pressure ", "Cloudy/Rain "
  UNSTABLE = 3,   // "Quickly rising H-Press",     "Not Stable"
  THUNDERSTORM = 4, // "Quickly falling L-Press",    "Thunderstorm"
  UNKNOWN = 5     // "Unknown (More Time needed)
};


// for forecast - untouched by me :)

  float lastPressure = -1;
  float lastTemp = -1;
  int lastForecast = -1;
  const int LAST_SAMPLES_COUNT = 5;
  float lastPressureSamples[LAST_SAMPLES_COUNT];
  // this CONVERSION_FACTOR is used to convert from Pa to kPa in forecast algorithm
  // get kPa/h be dividing hPa by 10
  #define CONVERSION_FACTOR (1.0/10.0)
  int minuteCount = 0;
  bool firstRound = true;
  // average value is used in forecast algorithm.
  float pressureAvg;
  // average after 2 hours is used as reference value for the next iteration.
  float pressureAvg2;
  float dP_dt;

// MyMessage to controler
MyMessage msgT1(CHILD_ID_TEMP, V_TEMP);
MyMessage msgP1(CHILD_ID_PRESS, V_PRESSURE);
MyMessage msgF1(CHILD_ID_PRESS, V_FORECAST);
MyMessage msgH1(CHILD_ID_HUM, V_HUM);

int batteryReportCounter = BATTERY_REPORT_CYCLE - 1;

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("OUTSIDE P&T&H", "1.2");
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_PRESS, S_BARO);
  present(CHILD_ID_HUM, S_HUM);  
}

void setup()
{
  delay(500);// just in case
  if (!bme.begin())
   {
    Serial.println("BME init failed!");
    while (1);
   }
  else Serial.println("BME init success!");
  
//  ServerUpdate(); // for first data reading and sending to controler
}

void loop()
{ 

  delay(10000);
  
  batteryReportCounter ++;
  bool forceTransmit = false;
  // Check battery
  if (batteryReportCounter >= BATTERY_REPORT_CYCLE) {
    long batteryVolt = readVcc();
    Serial.print("Battery voltage: "); Serial.print(batteryVolt); Serial.println(" mV");
    uint8_t batteryPcnt = constrain(map(batteryVolt,VMIN,VMAX,0,100),0,255);   
    Serial.print("Battery percent: "); Serial.print(batteryPcnt); Serial.println(" %");
    sendBatteryLevel(batteryPcnt);
    batteryReportCounter = 0;
  }
  
  double T, P, H;
  T=bme.readTemperature();
  P=bme.readPressure()/100.0;
  H=bme.readHumidity();
  
    int forecast = sample(P);
      send(msgT1.set(T, 1));
      send(msgP1.set(P, 1));
      send(msgH1.set(H,1));
      send(msgF1.set(weather[forecast]));
      
   // unmark for debuging
      Serial.print("T = \t"); Serial.print(T, 1); Serial.print(" degC\t");
      Serial.print("P = \t"); Serial.print(P, 1); Serial.print(" mBar\t");
      Serial.print("F = \t"); Serial.print(weather[forecast]); Serial.println(" ?");

    smartSleep(60000); // adjust sleeping time here 1 minute in this case 
}

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
  {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  return lastPressureSamplesAverage;
}

// Algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure)
{
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185)
  {
    minuteCount = 6;
  }

  if (minuteCount == 5)
  {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else
    {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) //first time initial 3 hour
    {
      dP_dt = change; //note this is for t = 1 hour
    }
    else
    {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else
    {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else
    {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    }
    else
    {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 3; // note this is for t = 3 hour
    }
    else
    {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  int forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
  {
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25))
  {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25)
  {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
  {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
  {
    forecast = SUNNY;
  }
  else if ((dP_dt > (-0.05)) && (dP_dt < 0.05))
  {
    forecast = STABLE;
  }
  else
  {
    forecast = UNKNOWN;
  }

  // uncomment when debugging
  //Serial.print(F("Forecast at minute "));
  //Serial.print(minuteCount);
  //Serial.print(F(" dP/dt = "));
  //Serial.print(dP_dt);
  //Serial.print(F("kPa/h --> "));
  //Serial.println(weather[forecast]);

  return forecast;
}

// function for reading Vcc by reading 1.1V reference against AVcc. Based from http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
// To calibrate reading replace 1125300L with scale_constant = internal1.1Ref * 1023 * 1000, where internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function) 
long readVcc() {
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
  long result = (high<<8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
