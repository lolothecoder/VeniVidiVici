/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE

#ifdef ESCON_MAXON
  void initMotorController() {
    digitalWrite(RIGHT_ENABLE, HIGH);
    digitalWrite(LEFT_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
    
    if (abs(spd) < MIN_PWM){
      spd = MIN_PWM;
    }
    else if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > MAX_PWM)
      spd = MAX_PWM;
    
    if (i == LEFT) { 
      if      (reverse == 0) { 
        analogWrite(LEFT_PWM, spd); 
        digitalWrite(LEFT_DIRECTION, 1); 
      }
      else if (reverse == 1) { 
        analogWrite(LEFT_PWM, spd); 
        digitalWrite(LEFT_DIRECTION, 0); 
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

#endif
#ifdef GRABBER
  void setGrabberSpeed (int spd){
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > MAX_GRABBER_PWM){
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
