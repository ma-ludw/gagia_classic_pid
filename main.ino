/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  This example shows how to use Arduino MKR 1010
  to connect your project to Blynk.

  Note: This requires WiFiNINA library
    from http://librarymanager/all#WiFiNINA

  Feel free to apply it to any other example. It's simple!
 *************************************************************/

/* Comment this out to disable prints and save space */
#define TEMP_OFFSET 7

#include <SPI.h>
#include <SAMD21PWM.h> //https://github.com/ocrdu/Arduino_SAMD21_turbo_PWM
#include <PID_v1.h> //https://playground.arduino.cc/Code/PIDLibrary/
#include <Adafruit_MAX31865.h>



const unsigned long PID_INTERVAL_MS = 1000;
unsigned long previousPidMillis = 0;
int count = 0;

// PID
double  actualTemp, controlValue;
double targetTemp = 94+TEMP_OFFSET;
double lastTemp = 0;
double Kp_heat = 8200;
double Ki_heat = 11000;
double Kd_heat = 18750;
double Kp_brew = 14000;
double Ki_brew = 23000;
double Kd_brew = 20000;
PID PIDTemp(&actualTemp, &controlValue, &targetTemp, Kp_heat, Ki_heat, Kd_heat, DIRECT);

// PWM
PWM pwm_heating = PWM(7, 64, 375000, false); // 1Hz PWM Frequenz on Pin 7

Adafruit_MAX31865 temp_sensor = Adafruit_MAX31865(8);

bool preheat = true;
bool brew = false;


void pid_task()
{
  uint16_t rtd;
  actualTemp = temp_sensor.temperature(100, 430);
  if(actualTemp)
  {
    //actualTemp = temp_sensor.temperatur(100, 430);
    if(lastTemp-actualTemp>0.3 && count == 0)
    {
      PIDTemp.SetTunings(Kp_brew,Ki_brew,Kd_brew);
      brew = true;
    }
    else if(count>25)
    {
      PIDTemp.SetTunings(Kp_heat,Ki_heat,Kd_heat);
      count = 0;
      brew = false;
    }
    if(brew)
    {
      count++;
    }
    lastTemp = actualTemp;
    computePID();
    pwm_heating.dutyCycle((int)(controlValue/375000.0*1000.0));
  }

  Serial.print("ControlValue: "); Serial.print((int)(controlValue/375000.0*1000.0)); Serial.print("\n\r");
  Serial.print("Temp: "); Serial.print((int)actualTemp); Serial.print("\n\r");
}

void computePID()
{

  // read Temperature
  // Da es beim Einschalten der Maschine eine Zeit dauert bis sie heiss ist, besteht die Gefahr das der I-Anteil zu gross wird und so das Wasser zu heiss wird. Um das zu verhindern, kann die Zeitkostante des Reglers vergrössert werden,
  // dies würde aber zu einem langsamen Regler führen, was schlecht währe, wenn ein Kaffeebezug gestartet wird, da so die Temperatur während des Bezuges nicht konstannt gehalten werden kann. Ein bessere Möglichkeit ist, die Heizung
  // bis zu einer Temperatur von z.B. 80°C dauerhaft einzuschalten und der Regler erst ab dieser Temperatur. So kann er schneller ausgelegt werden, da die Temperaturdifferenz zwischen soll und ist kleiner ist.
  // Besser Integral Windup auch in library enthalten

  if(preheat)
  {
    Serial.print("Preheat: ");Serial.print((int)actualTemp);Serial.print("\n\r");
    controlValue = 375000;
  }
  else
  {
    PIDTemp.Compute();
    Serial.print("PID: ");Serial.print((int)actualTemp);Serial.print("\n\r");
    Serial.print("PID Control: ");Serial.print((int)controlValue);Serial.print("\n\r");
  }
  if(targetTemp-actualTemp < 15 && preheat)
  {
    preheat = false;
  }
  else if(targetTemp-actualTemp > 30 && preheat == false)
  {
    preheat = true;
  }
}


void setup()
{
  // Debug console
  Serial.begin(9600);
  
  //while (!Serial);
  // delay(10000);

  Serial.println("Started\n\r");

  // initialize the temp sensor
  temp_sensor.begin(MAX31865_3WIRE);

  //PID setup
  PIDTemp.SetOutputLimits(0,375000); // min max value of PWM
  PIDTemp.SetMode(AUTOMATIC);
  PIDTemp.Compute();

  //PWM setup
  pwm_heating.dutyCycle(500);
  pwm_heating.enable(true);

  Serial.print("PWM Frequency: "); Serial.print((int)pwm_heating.frequency()); Serial.print("\n\r");

  previousPidMillis = millis();
}

void loop()
{
  const unsigned long currentMillis = millis();
  if (currentMillis - previousPidMillis >= PID_INTERVAL_MS)
  {
    previousPidMillis = currentMillis;
    pid_task();
  }
}
