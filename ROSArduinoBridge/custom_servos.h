#ifdef CUSTOM_SERVOS

#define DOOR_PWM 6
#define BACKRAMP_PWM 11

#define DOOR_CLOSE 0
#define DOOR_OPEN_SHORTCUT 1
#define DOOR_OPEN 70

#define RAMP_DOWN 0
#define RAMP_UP_SHORTCUT 1
#define RAMP_UP 20

#define DOOR_ID 0
#define  RAMP_ID 1

Servo door;  
Servo ramp;

#endif

void initServos();
void setServo(int i, int pos);