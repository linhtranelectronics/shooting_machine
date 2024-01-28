
#define STEPMAX 2900
#define ANGLEMAX 125

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>
#include <JC_Button.h>
#include <MsTimer2.h>
#include <TimerOne.h>

#define SERVOMIN 125
#define SERVOMAX 575
#define STEP_PIN 9
#define DIR_PIN 10
#define EN_PIN 11
#define ENSTOP_PIN 14
#define BTNA_PIN 2
#define BTNB_PIN 3
#define BTNC_PIN 4
#define BTND_PIN 5
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
AccelStepper stepper(2, STEP_PIN, DIR_PIN);
Button btnA(BTNA_PIN);
Button btnB(BTNB_PIN);
Button btnC(BTNC_PIN);
Button btnD(BTND_PIN);

int16_t pwmValCurrent[16];
int16_t pwmValSet[16];
int16_t speedChange[16];
uint8_t lessonNum = 0;
void setup()
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  btnA.begin();
  btnB.begin();
  btnC.begin();
  btnD.begin();
  speedChangePWM(9, 100);
  pwm.begin();
  pwm.reset();
  delay(100);
  pwm.begin();
  pwm.setPWMFreq(60);
  Serial.print("hello");
  for (int i = 0; i < 8; i++)
  {
    pwm.setPWM(i, 0, angleToPulse(0));
    delay(1);
  }
  for (int i = 8; i < 16; i++)
  {
    pwm.setPWM(i, 0, 0);
    delay(1);
  }
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();
  stepper.setMaxSpeed(300000);
  stepper.setAcceleration(5000.0);
  home();
  Serial.println("start");
}
void loop()
{
  static uint32_t lastupdate;
  if (millis() >= lastupdate + 20)
  {
    lastupdate = millis();
    updatePWM();
  }
  
  readBtn();
  stepper.run();
  switch (lessonNum)
  {
  case 0:
    lesson0();
    break;
  case 1:
    lesson1();
    break;
  case 2:
    lesson2();
    break;
  case 3:
    lesson3();
    break;
  case 4:
    lesson4();
    break;
  case 5:
    lesson5();
    break;

  default:
    break;
  }
}
void readBtn()
{
  btnA.read();
  btnB.read();
  btnC.read();
  btnD.read();

  if (btnA.wasPressed())
  {
    lessonNum =1; 
  }
  if (btnA.pressedFor(2000) && lessonNum != 0)
  {
   lessonNum = 0;
  }
  
  
  if (btnA.pressedFor(2000) && lessonNum != 0)
  {
    lessonNum = 0;
  }
  if (btnB.wasPressed())
  {
    lessonNum = 2;
  }
  if (btnC.wasPressed())
  {
    lessonNum = 3;
  }
  if (btnD.wasPressed())
  {
    lessonNum = 4;
  }
  if (btnD.pressedFor(2000) && lessonNum != 5)
  {
    lessonNum = 5;
  }
}

int angleToPulse(int ang)
{
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX); // map angle of 0 to 180 to Servo min and Servo max
  return pulse;
}
void changePWM(int pin, int duty)
{
  pwmValSet[pin] = duty;
}
bool checkPWM()
{
  for (int i = 0; i < 16; i++)
  {
    if (pwmValCurrent[i] != pwmValSet[i])
    {
      return false;
    }
  }
  return true;
}
void speedChangePWM(uint8_t lowsp, uint8_t highsp)
{
  for (uint8_t i = 0; i < 16; i++)
  {
    speedChange[i] = lowsp;
    if (i >= 8)
    {
      speedChange[i] = highsp;
    }
  }
}
void updatePWM(void)
{
  digitalWrite(13, !digitalRead(13));
  for (uint8_t i = 0; i < 16; i++)
  {

    if (pwmValCurrent[i] > pwmValSet[i])
    {
      pwmValCurrent[i] = pwmValCurrent[i] - speedChange[i];
      if (pwmValCurrent[i] < pwmValSet[i])
      {
        pwmValCurrent[i] = pwmValSet[i];
      }
      pwm.setPWM(i, 0, pwmValCurrent[i]);
    }
    else if (pwmValCurrent[i] < pwmValSet[i])
    {
      pwmValCurrent[i] = pwmValCurrent[i] + speedChange[i];
      if (pwmValCurrent[i] > pwmValSet[i])
      {
        pwmValCurrent[i] = pwmValSet[i];
      }
      pwm.setPWM(i, 0, pwmValCurrent[i]);
    }
  }
}

void home()
{

  int homingPos = 0;
  pinMode(ENSTOP_PIN, INPUT);
  stepper.setMaxSpeed(10000);
  stepper.setAcceleration(500.0);
  while (digitalRead(ENSTOP_PIN) == 0)
  {
    stepper.moveTo(homingPos);
    homingPos -= 5;
    stepper.run();
    delay(1);
  }
  stepper.stop();
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(100000);
  stepper.setAcceleration(500.0);
}
void lesson0()
{
  resetServo();
  stepper.moveTo(0);
  stepper.runToPosition();
}
void lesson1()
{
  changePWM(0, angleToPulse(ANGLEMAX));
  changePWM(1, angleToPulse(0));
  changePWM(2, angleToPulse(0));
  changePWM(3, angleToPulse(0));

  changePWM(4, angleToPulse(ANGLEMAX));
  changePWM(5, angleToPulse(0));
  changePWM(6, angleToPulse(0));
  changePWM(7, angleToPulse(0));

  changePWM(8, 4000);
  changePWM(9, 0);
  changePWM(10, 0);
  changePWM(11, 0);
  if (checkPWM())
  {
    stepper.moveTo(STEPMAX);
    while (stepper.distanceToGo() != 0)
    {
      stepper.run();
    }
  }
}

void lesson2()
{
  changePWM(0, angleToPulse(0));
  changePWM(1, angleToPulse(ANGLEMAX));
  changePWM(2, angleToPulse(0));
  changePWM(3, angleToPulse(0));

  changePWM(4, angleToPulse(0));
  changePWM(5, angleToPulse(ANGLEMAX));
  changePWM(6, angleToPulse(0));
  changePWM(7, angleToPulse(0));

  changePWM(8, 0);
  changePWM(9, 4000);
  changePWM(10, 0);
  changePWM(11, 0);
}
void lesson3()
{
  changePWM(0, angleToPulse(0));
  changePWM(1, angleToPulse(0));
  changePWM(2, angleToPulse(ANGLEMAX));
  changePWM(3, angleToPulse(0));

  changePWM(4, angleToPulse(0));
  changePWM(5, angleToPulse(0));
  changePWM(6, angleToPulse(ANGLEMAX));
  changePWM(7, angleToPulse(0));

  changePWM(8, 0);
  changePWM(9, 0);
  changePWM(10, 4000);
  changePWM(11, 0);
}
void lesson4()
{
  changePWM(0, angleToPulse(0));
  changePWM(1, angleToPulse(0));
  changePWM(2, angleToPulse(0));
  changePWM(3, angleToPulse(ANGLEMAX));

  changePWM(4, angleToPulse(0));
  changePWM(5, angleToPulse(0));
  changePWM(6, angleToPulse(0));
  changePWM(7, angleToPulse(ANGLEMAX));

  changePWM(8, 0);
  changePWM(9, 0);
  changePWM(10, 0);
  changePWM(11, 4000);
}
void lesson5()
{
  changePWM(0, angleToPulse(ANGLEMAX));
  changePWM(1, angleToPulse(ANGLEMAX));
  changePWM(2, angleToPulse(ANGLEMAX));
  changePWM(3, angleToPulse(ANGLEMAX));

  changePWM(4, angleToPulse(ANGLEMAX));
  changePWM(5, angleToPulse(ANGLEMAX));
  changePWM(6, angleToPulse(ANGLEMAX));
  changePWM(7, angleToPulse(ANGLEMAX));
  changePWM(8, 4000);
  changePWM(9, 4000);
  changePWM(10, 4000);
  changePWM(11, 4000);
}
void resetServo()
{
  for (int i = 0; i < 12; i++)
  {
    changePWM(i, angleToPulse(0));
  }
}
