/**
   \par Copyright (C), 2012-2017, MakeBlock
   @file    SmartServoTestRGBLedColorLoop.ino
   @author  MakeBlock
   @version V1.0.0
   @date    2017/03/04
   @brief   Description: this file is sample code for Smart servo device.

   Function List:
   1.  boolean MSmartServo::assignDevIdRequest(void);
   2.  boolean MSmartServo::moveTo(uint8_t dev_id,long angle_value,float speed);
   3.  boolean MSmartServo::move(uint8_t dev_id,long angle_value,float speed);
   4.  long MSmartServo::getAngleRequest(uint8_t devId);
   5.  float MSmartServo::getSpeedRequest(uint8_t devId);
   6.  float MSmartServo::getVoltageRequest(uint8_t devId);
   7.  float MSmartServo::getTempRequest(uint8_t devId);
   8.  float MSmartServo::getCurrentRequest(uint8_t devId);
   9.  boolean MSmartServo::setZero(uint8_t dev_id);
   10. boolean MSmartServo::setBreak(uint8_t dev_id, uint8_t breakStatus);
   11. boolean MSmartServo::setRGBLed(uint8_t dev_id, uint8_t r_value, uint8_t g_value, uint8_t b_value);
   12. boolean MSmartServo::setPwmMove(uint8_t dev_id, int16_t pwm_value);
   13. boolean MSmartServo::setInitAngle(uint8_t dev_id);

   \par History:
   <pre>
   <Author>     <Time>        <Version>      <Descr>
   Watson     2017/03/04    1.0.0          build the new
   </pre>
*/

#include <MSmartServo.h>

MSmartServo mysmartservo(2); //use hardware serial

float j, f, k;

void setup()
{
  mysmartservo.begin(115200);
  delay(5);
  mysmartservo.assignDevIdRequest();
  delay(50);//must delay over 50ms
}

void loop()
{
  color_loop();
}

void color_loop()
{
  uint8_t red = 64 * (1 + sin(1 / 2.0 + j / 4.0) );
  uint8_t green = 64 * (1 + sin(1 / 1.0 + f / 9.0 + 2.1) );
  uint8_t blue = 64 * (1 + sin(1 / 3.0 + k / 14.0 + 4.2) );

  mysmartservo.setRGBLed(1,red, green, blue);//device ID, red, greed, blue;

  j += random(1, 6) / 6.0;
  f += random(1, 6) / 6.0;
  k += random(1, 6) / 6.0;
}
