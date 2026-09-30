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
#define BLYNK_PRINT Serial
//#define USE_TIMER_3     true
#define TEMP_OFFSET 7
#define HW_TIMER_INTERVAL_MS      5


#include <SPI.h>
#include <WiFiNINA.h>
#include <BlynkSimpleWiFiNINA.h>
#include <FlashAsEEPROM.h> //https://github.com/cmaglie/FlashStorage
#include <FlashStorage.h>
#include <SAMD21PWM.h> //https://github.com/ocrdu/Arduino_SAMD21_turbo_PWM
#include <PID_v1.h> //https://playground.arduino.cc/Code/PIDLibrary/
#include <Adafruit_MAX31865.h>
#include "SAMDTimerInterrupt.h"
#include "SAMD_ISR_Timer.h"

#include "credentials.h"


BlynkTimer timer;
int timerid;
int periodms = 1000;
int count = 0;

FlashStorage(setTempFlash, float);

// PID
double targetTemp, actualTemp, controlValue;
double lastTemp = 0;
double Kp_heat = 8200;
double Ki_heat = 11000;
double Kd_heat = 18750;
double Kp_brew = 14000;
double Ki_brew = 23000;
double Kd_brew = 20000;
PID PIDTemp(&actualTemp, &controlValue, &targetTemp, Kp_heat, Ki_heat, Kd_heat, DIRECT);

// PWM
PWM pwm_heizung = PWM(7,64,375000,false); // 1Hz PWM Frequenz on Pin 16 (A2)

Adafruit_MAX31865 temp_sensor = Adafruit_MAX31865(8);

bool preheat = true;
bool brew = false;

SAMDTimer ITimer(TIMER_TC3);
SAMD_ISR_Timer ISR_Timer;


BLYNK_WRITE(V1)
{
  targetTemp = param.asFloat()+TEMP_OFFSET; // assigning incoming value from pin V1 to a variable
  setTempFlash.write(targetTemp);  // writing targetTemp to flash
}
void TimerHandler(void)
{
  ISR_Timer.run();
}
void pid_task()
{
  uint16_t rtd;
  if(temp_sensor.readRTDAsync(&rtd))
  {
    actualTemp = temp_sensor.temperatureAsync(rtd, 100, 430);
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
    pwm_heizung.dutyCycle((int)(controlValue/375000*1000));
  }
  else
  {
    ISR_Timer.setTimeout(11, pid_task);
  }

  //Serial.print((int)(controlValue/375000*1000));Serial.print("Temp:");Serial.print((int)actualTemp);Serial.print("\n");
  //Serial.print((int)pwm_heizung.frequency());Serial.print("\n");
}

void sendTemp()
{
  Blynk.virtualWrite(V2,actualTemp-TEMP_OFFSET);
  Blynk.virtualWrite(V3,(int)(controlValue/375000*1000));
  Serial.print((int)(controlValue/375000*1000));Serial.print("Temp:");Serial.print((int)actualTemp);Serial.print("\n");
}

void computePID()
{

  // read Temperature
  // Da es beim Einschalten der Maschine eine Zeit dauert bis sie heiss ist, besteht die Gefahr das der I-Anteil zu gross wird und so das Wasser zu heiss wird. Um das zu verhindern, kann die Zeitkostante des Reglers vergrössert werden,
  // dies würde aber zu einem langsamen Regler führen, was schlecht währe, wenn ein Kaffeebezug gestartet wird, da so die Temperatur während des Bezuges nicht konstannt gehalten werden kann. Ein Bessere Möglichkeit weähre, die Heizung
  // bis zu einer Temperatur von z.B. 80°C dauerhaft einzuschalten und der Regler erst ab dieser Temperatur. So kann er schneller ausgelegt werden, da die Temperaturdifferenz zwischen soll und ist kleiner ist.
  // Besser Integral Windup auch in library enthalten

  if(preheat)
  {
    controlValue = 375000;
  }
  else
  {
    PIDTemp.Compute();
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

  // initialize digital pin LED_BUILTIN as an output.
//  pinMode(LED_BUILTIN, OUTPUT);


  // initialize the temp sensor
  temp_sensor.begin(MAX31865_3WIRE);

  targetTemp = setTempFlash.read();

  //PID setup
  PIDTemp.SetOutputLimits(0,375000); // min max value of PWM
  PIDTemp.SetMode(AUTOMATIC);
  PIDTemp.Compute();

  //PWM setup
  pwm_heizung.dutyCycle(500);
  pwm_heizung.enable(true);

  //targetTemp = 101;
   

  timerid = timer.setInterval((int)(1000),sendTemp);

  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS*1000, TimerHandler))
  {
    Serial.print(F("Starting ITimer OK, millis() = ")); Serial.println(millis());
  }
  else
  {
    Serial.println("Can't set ITimer correctly. Select another freq. or interval");
  }
  ISR_Timer.setInterval(periodms, pid_task);

}

void loop()
{
  Blynk.run();
  timer.run();
}
