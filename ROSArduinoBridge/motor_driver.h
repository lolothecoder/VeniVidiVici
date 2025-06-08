/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef ESCON_MAXON

#define RIGHT_ENABLE 24
#define RIGHT_DIRECTION 25
#define RIGHT_PWM 4

#define LEFT_ENABLE 26
#define LEFT_DIRECTION 27
#define LEFT_PWM 5

#define MIN_PWM 25
#define MAX_PWM 220
#define MAX_SPEED 7000
#endif 

#ifdef GRABBER

#define GRABBER_DIRECTION 22
#define GRABBER_PWM 3
#define MAX_GRABBER_PWM 120

#endif


void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

void setGrabberSpeed (int spd);
