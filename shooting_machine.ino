
#define STEPMAX 1000
#define ANGLEMAX 125

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>
#include <JC_Button.h>

#define SERVOMIN 125
#define SERVOMAX 575
#define STEP_PIN 12
#define DIR_PIN 11
#define EN_PIN 10
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

static uint16_t pwmValCurrent[16];
static uint16_t pwmValSet[16];

void setup()
{
  Serial.begin(115200);
  btnA.begin();
  btnB.begin();
  btnC.begin();
  btnD.begin();
  pwm.begin();
  pwm.reset();
  delay(100);
  pwm.begin();
  pwm.setPWMFreq(60);
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
  stepper.setMaxSpeed(100000);
  stepper.setAcceleration(500.0);
  home();
  Serial.println("start");
}
void loop()
{
  btnA.read();
  btnB.read();
  btnC.read();
  btnD.read();
  if (btnA.releasedFor(2000))  
  {
    Serial.println("reset all");
    resetServo();
    stepper.moveTo(0);
    stepper.runToPosition();
    
  }
  
  if (btnA.wasReleased())
  {
    Serial.println("lesson 1");
    lesson1();
  }
  if (btnB.wasReleased())
  {
    Serial.println("lesson 2");
    lesson2();
  }

  if (btnC.wasReleased())
  {
    Serial.println("lesson 3");
    lesson3();
  }
  if (btnD.wasReleased())
  {
    Serial.println("lesson 4");
    lesson4();
  }
  // stepper.run();
  updatePWM();
}

int angleToPulse(int ang)
{
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX); // map angle of 0 to 180 to Servo min and Servo max
  // Serial.print("Angle: ");
  // Serial.print(ang);
  // Serial.print(" pulse: ");
  // Serial.println(pulse);
  return pulse;
}

void changePWM(int pin, int duty)
{
  pwmValSet[pin] = duty;
}
void updatePWM()
{
  static uint16_t speedChange[16];
  for (uint8_t i = 0; i < sizeof(pwmValCurrent) / sizeof(uint16_t); i++)
  {
    speedChange[i] = 9;
    if (i >= 8)
    {
      speedChange[i] = 100;
    }
  }

  uint8_t timeChange = 20;
  static uint32_t lastTimeChange;

  if (lastTimeChange + timeChange <= millis())
  {
    lastTimeChange = millis();
    for (uint8_t i = 0; i < sizeof(pwmValCurrent) / sizeof(uint16_t); i++)
    {
      if (pwmValCurrent[i] > pwmValSet[i])
      {
        pwmValCurrent[i] = pwmValCurrent[i] - speedChange[i];
        if (pwmValCurrent[i] < pwmValSet[i])
        {
          pwmValCurrent[i] = pwmValSet[i];
        }
      }
      else if (pwmValCurrent[i] < pwmValSet[i])
      {
        pwmValCurrent[i] = pwmValCurrent[i] + speedChange[i];
        if (pwmValCurrent[i] > pwmValSet[i])
        {
          pwmValCurrent[i] = pwmValSet[i];
        }
      }
      pwm.setPWM(i, 0, pwmValCurrent[i]);
    }
  }
}

void home()
{

  int homingPos = 0;
  pinMode(ENSTOP_PIN, INPUT);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500.0);
  while (digitalRead(ENSTOP_PIN) == 0)
  {
    stepper.moveTo(homingPos);
    homingPos -= 1;
    stepper.run();
    delay(5);
    updatePWM();
  }
  stepper.stop();
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(100000);
  stepper.setAcceleration(500.0);
}

void lesson1()
{
  stepper.moveTo(STEPMAX);
  while (stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  changePWM(0, angleToPulse(ANGLEMAX));
  changePWM(1, angleToPulse(0));
  changePWM(2, angleToPulse(0));

  changePWM(4, angleToPulse(ANGLEMAX));
  changePWM(5, angleToPulse(0));
  changePWM(6, angleToPulse(0));
  

  changePWM(8, 4000);
  changePWM(9, 0);
  changePWM(10, 0);
}
void lesson2()
{
  changePWM(0, angleToPulse(0));
  changePWM(1, angleToPulse(ANGLEMAX));
  changePWM(2, angleToPulse(0));

  changePWM(4, angleToPulse(0));
  changePWM(5, angleToPulse(ANGLEMAX));
  changePWM(6, angleToPulse(0));
  

  changePWM(8, 0);
  changePWM(9, 4000);
  changePWM(10, 0);
}
void lesson3()
{
  changePWM(0, angleToPulse(0));
  changePWM(1, angleToPulse(0));
  changePWM(2, angleToPulse(ANGLEMAX));

  changePWM(4, angleToPulse(0));
  changePWM(5, angleToPulse(0));
  changePWM(6, angleToPulse(ANGLEMAX));
  

  changePWM(8, 0);
  changePWM(9, 0);
  changePWM(10, 4000);
}
void lesson4()
{
  changePWM(0, angleToPulse(ANGLEMAX));
  changePWM(1, angleToPulse(ANGLEMAX));
  changePWM(2, angleToPulse(ANGLEMAX));

  changePWM(4, angleToPulse(ANGLEMAX));
  changePWM(5, angleToPulse(ANGLEMAX));
  changePWM(6, angleToPulse(ANGLEMAX));
  

  changePWM(8, 4000);
  changePWM(9, 4000);
  changePWM(10, 4000);
}
void resetServo()
{
  changePWM(0, angleToPulse(0));
  changePWM(1, angleToPulse(0));
  changePWM(2, angleToPulse(0));

  changePWM(4, angleToPulse(0));
  changePWM(5, angleToPulse(0));
  changePWM(6, angleToPulse(0));
  

  changePWM(8, 0);
  changePWM(9, 0);
  changePWM(10, 0);
}