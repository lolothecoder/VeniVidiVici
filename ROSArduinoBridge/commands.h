/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define READ_ENCODERS  'e'
#define BATTERY_LEVEL  'f'
#define GRABBER_RAW_PWM'g'
#define ACTIVATE_SERVO 'h'
#define GET_INFO       'i'
#define GET_IMU        'j' // Deprecated, use GET_INFO instead
#define MOTOR_RAW_PWM  'o'
#define LEFT            0
#define RIGHT           1

#endif


