#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>


#define SERVOMIN  125 
#define SERVOMAX  575 
#define STEP_PIN 12
#define DIR_PIN 11
#define EN_PIN 10


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
AccelStepper stepper(1, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60);
  pwm.setPWM(0, 0, angleToPulse(0)); 
  pwm.setPWM(15, 0, 0); 
  //stepper.setEnablePin(EN_PIN);
  //stepper.enableOutputs();
  // stepper.setSpeed(10000000);
  // stepper.setMaxSpeed(5000);
  // stepper.setAcceleration(2000.0);
  // stepper.moveTo(2000);
}
void loop() {
      if (stepper.distanceToGo() == 0)
    {
	// Random change to speed, position and acceleration
	// Make sure we dont get 0 speed or accelerations
	delay(1000);
	stepper.moveTo(rand() % 200);
	stepper.setMaxSpeed((rand() % 200) + 1);
	stepper.setAcceleration((rand() % 200) + 1);
    }
stepper.run();
changePWM(0, angleToPulse(125));
changePWM(15, 4095);
}
int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   Serial.print("Angle: ");Serial.print(ang);
   Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}
void changePWM(int pin, int duty)
{
  static uint16_t pwmValCurrent[16];
  static uint16_t pwmValSet[16];
  static uint16_t speedChange[16];
  for (uint8_t i = 0; i < sizeof(pwmValCurrent)/ sizeof(uint16_t); i++)
  {
    speedChange[i] = 10;
    if (i >= 12)
    {
      speedChange[i] = 20;
    }
    
  }
  
  uint8_t timeChange = 2;
  static uint32_t lastTimeChange;
  pwmValSet[pin] = duty;
  if (lastTimeChange + timeChange <= millis())
  {
    lastTimeChange = millis();
    for (uint8_t i = 0; i < sizeof(pwmValCurrent)/ sizeof(uint16_t); i++)
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