/*#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;*/

// Project parameters
#define STATUS_LED_PIN 13
#define BATTERY_VOLTAGE_PIN A0
const float NOMINAL_BATTERY_VOLTAGE = 12.0;

// Sensor data
float batteryVoltage = 0; // V
float batteryLevel = 0; // %
float temperature = 0; // degC
float altitude = 0; // m

// Internal use variables
bool phoneConnected = false;
unsigned long lastLedBlink=0;
unsigned long lastBatteryLevelUpdate=0;
unsigned long lastTempAndAltUpdate=0;

void setup()
{
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  Serial.begin(9600);

  /*if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }*/
}

void loop()
{
  if(!phoneConnected && TimedRun(500, &lastLedBlink))
    BlinkLed();
  if(TimedRun(1000, &lastBatteryLevelUpdate))
  {
  	batteryVoltage = GetBatteryVoltage();
    // Assumes linear power discharge
    batteryLevel = batteryVoltage/NOMINAL_BATTERY_VOLTAGE*100.0;
  }
  /*if(TimedRun(1000, &lastTempAndAltUpdate))
  {
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    temperature = bme.readTemperature();
  }*/
  
  SerialDebug();
  
  
  delay(100);
}

// Gets battery level at specific frequency
float GetBatteryVoltage()
{
  int portRead = analogRead(BATTERY_VOLTAGE_PIN);
  return (portRead/1023.0)*NOMINAL_BATTERY_VOLTAGE;
}

// Toggles led at specified frequency
bool isLedOn=true;
void BlinkLed()
{
  if(isLedOn)
    digitalWrite(STATUS_LED_PIN, LOW);
  else
    digitalWrite(STATUS_LED_PIN, HIGH);
  isLedOn = !isLedOn;
}

// Returns when the periodic section of the code should run
bool TimedRun(unsigned long msInterval, unsigned long *lastCall)
{
  if( millis()-*lastCall >= msInterval )
  {
    *lastCall = millis();
    return true;
  }
  return false;
}

void SerialDebug()
{
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("*C. Altitude: ");
  Serial.print(altitude);
  Serial.print("m. Battery: ");
  Serial.print(batteryVoltage);
  Serial.print("V (");
  Serial.print(batteryLevel);
  Serial.println("%).");
}