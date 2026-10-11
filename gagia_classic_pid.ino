#define TEMP_OFFSET 7

#include <SPI.h>
#include <SAMD21PWM.h> // https://github.com/ocrdu/Arduino_SAMD21_turbo_PWM
#include <PID_v1.h> // https://playground.arduino.cc/Code/PIDLibrary/
#include <Adafruit_MAX31865.h>

namespace
{
const uint32_t PID_INTERVAL_MS = 1000UL;
const double PWM_MAX_VALUE = 375000.0;
const double PREHEAT_MAX_OUTPUT = PWM_MAX_VALUE;
const double TEMPERATURE_DELTA_FOR_BREW = 0.3;
const uint8_t BREW_DETECTION_LIMIT = 25;
const double PREHEAT_EXIT_THRESHOLD_C = 15.0;
const double PREHEAT_REENTRY_THRESHOLD_C = 30.0;

const double TARGET_TEMPERATURE_C = 94.0 + TEMP_OFFSET;

const double KP_HEAT = 8200.0;
const double KI_HEAT = 11000.0;
const double KD_HEAT = 18750.0;
const double KP_BREW = 14000.0;
const double KI_BREW = 23000.0;
const double KD_BREW = 20000.0;

const uint8_t HEATING_PWM_PIN = 7;
const uint8_t TEMPERATURE_SENSOR_CS_PIN = 8;

double actualTemp = 0.0;
double controlValue = 0.0;
double targetTemp = TARGET_TEMPERATURE_C;
double lastTemp = 0.0;
unsigned long previousPidMillis = 0;

bool preheat = true;
bool brew = false;
uint8_t brewCount = 0;

PID pidController(&actualTemp, &controlValue, &targetTemp, KP_HEAT, KI_HEAT, KD_HEAT, DIRECT);
PWM pwmHeating = PWM(HEATING_PWM_PIN, 64, 375000, false);
Adafruit_MAX31865 temperatureSensor = Adafruit_MAX31865(TEMPERATURE_SENSOR_CS_PIN);
}

void applyOutputToHeater()
{
  const int dutyCycle = static_cast<int>((controlValue / PWM_MAX_VALUE) * 1000.0);
  pwmHeating.dutyCycle(dutyCycle);
}

void updateBrewState()
{
  const bool droppingTemperature = (lastTemp - actualTemp > TEMPERATURE_DELTA_FOR_BREW);

  if (droppingTemperature && brewCount == 0)
  {
    pidController.SetTunings(KP_BREW, KI_BREW, KD_BREW);
    brew = true;
  }
  else if (brewCount > BREW_DETECTION_LIMIT)
  {
    pidController.SetTunings(KP_HEAT, KI_HEAT, KD_HEAT);
    brewCount = 0;
    brew = false;
  }

  if (brew)
  {
    brewCount++;
  }

  lastTemp = actualTemp;
}

void updateTemperatureControl()
{
  if (preheat)
  {
    Serial.print(F("Preheat: "));
    Serial.print(static_cast<int>(actualTemp));
    Serial.print(F("\n\r"));
    controlValue = PREHEAT_MAX_OUTPUT;
  }
  else
  {
    pidController.Compute();
    Serial.print(F("PID: "));
    Serial.print(static_cast<int>(actualTemp));
    Serial.print(F("\n\r"));
    Serial.print(F("PID Control: "));
    Serial.print(static_cast<int>(controlValue));
    Serial.print(F("\n\r"));
  }

  if ((targetTemp - actualTemp) < PREHEAT_EXIT_THRESHOLD_C && preheat)
  {
    preheat = false;
  }
  else if ((targetTemp - actualTemp) > PREHEAT_REENTRY_THRESHOLD_C && !preheat)
  {
    preheat = true;
  }
}

void pidTask()
{
  actualTemp = temperatureSensor.temperature(100, 430);

  if (actualTemp > 0.0)
  {
    updateBrewState();
    updateTemperatureControl();
    applyOutputToHeater();
  }

  Serial.print(F("ControlValue: "));
  Serial.print(static_cast<int>((controlValue / PWM_MAX_VALUE) * 1000.0));
  Serial.print(F("\n\r"));

  Serial.print(F("Temp: "));
  Serial.print(static_cast<int>(actualTemp));
  Serial.print(F("\n\r"));
}

void setup()
{
  Serial.begin(9600);
  Serial.println(F("Started\n\r"));

  temperatureSensor.begin(MAX31865_3WIRE);

  pidController.SetOutputLimits(0, static_cast<double>(PWM_MAX_VALUE));
  pidController.SetMode(AUTOMATIC);
  pidController.Compute();

  pwmHeating.dutyCycle(500);
  pwmHeating.enable(true);

  Serial.print(F("PWM Frequency: "));
  Serial.print(static_cast<int>(pwmHeating.frequency()));
  Serial.print(F("\n\r"));

  previousPidMillis = millis();
}

void loop()
{
  const unsigned long currentMillis = millis();

  if (currentMillis - previousPidMillis >= PID_INTERVAL_MS)
  {
    previousPidMillis = currentMillis;
    pidTask();
  }
}
