/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
   
#ifdef POLOLU_VNH5019
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined ESCON_MAXON
  void initMotorController() {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > MAX_PWM)
      spd = MAX_PWM;
    
    if (i == LEFT) { 
      if      (reverse == 0) { 
        analogWrite(LEFT_PWM, spd); 
        digitalWrite(LEFT_DIRECTION, 0); 
      }
      else if (reverse == 1) { 
        analogWrite(LEFT_PWM, spd); 
        digitalWrite(LEFT_DIRECTION, 1); 
      }
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if (reverse == 0) { 
        analogWrite(RIGHT_PWM, spd); 
        digitalWrite(RIGHT_DIRECTION, 0); 
      }
      else if (reverse == 1) { 
        analogWrite(RIGHT_PWM, spd); 
        digitalWrite(RIGHT_DIRECTION, 1); 
      }
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#else
  #error A motor driver must be selected!
#endif

#ifdef GRABBER
  void setGrabberSpeed (int speed){
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > MAX_GRABBER_PWM)
      spd = MAX_GRABBER_PWM;
    }

    if(reverse == 0){
      analogWrite(GRABBER_PWM, spd); 
      digitalWrite(GRABBER_DIRECTION, 0); 
    }
    else if (reverse == 1){
      analogWrite(GRABBER_PWM, spd); 
      digitalWrite(GRABBER_DIRECTION, 1); 
    }
  }
#endif
#endif
