#ifdef CUSTOM_SERVOS
#include <Servo.h>
void initServos(){
    door.attach(DOOR_PWM);
    ramp.attach(BACKRAMP_PWM);
    door.write(DOOR_CLOSE);
    ramp.write(RAMP_DOWN);
}
void setServo(int i, int pos){
    if (i == DOOR_ID){
        if (pos == DOOR_CLOSE){
            door.write(DOOR_CLOSE);
        } else if (pos == DOOR_OPEN_SHORTCUT){
            door.write(DOOR_OPEN);
        } else {
             Serial.print("Not Valid");
        }
    } else if (i == RAMP_ID){
         if (pos == RAMP_DOWN){
            ramp.write(RAMP_DOWN);
        } else if (pos == RAMP_UP_SHORTCUT){
            ramp.write(RAMP_UP);
        } else {
             Serial.print("Not Valid");
        }
    } else {
         Serial.print("Not Valid");
    }
}

#endif