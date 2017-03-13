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

MSmartServo mysmartservo;  //use hardware serial

void setup()
{
  mysmartservo.begin(115200);
  delay(5);
  mysmartservo.assignDevIdRequest();
  delay(1000);//must delay over 50ms
  mysmartservo.setZero(1);
  delay(40);
}

void loop()
{
  mysmartservo.setPwmMove(1, 254); //device ID, speed;
  delay(3000);
  mysmartservo.setPwmMove(1, 0); //device ID, speed;
  delay(500);
  mysmartservo.setPwmMove(1, -254); //device ID, speed;
  delay(3000);
  mysmartservo.setPwmMove(1, 0); //device ID, speed;
  delay(500);
}

